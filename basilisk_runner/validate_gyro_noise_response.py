"""8A-2B live discrete gyro-noise response, ASSUMED / TEST-ONLY, 2026-10-02.

Reuse the committed three-second case and 8A-2A read-only witnesses. No sensor,
estimator, controller, Q/R/P0 or runtime wiring changes. The independent PCG64
sequence tests delivery, not NumPy's Gaussian distribution (already tested in
8A-1). Interpolated gyro substeps are correlated, not new noise acquisitions.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from attitude_mekf_adapter import ShadowOptions
from gyro_sensor_model import profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_gyro_bias_response import (
    DURATION_S, HERE, ROOT, ROUND_OFF, characterize, json_value, run_case,
)
from validate_mekf_closed_loop import csv_bytes, vector

SCOPE = "SYNTHETIC GYRO-NOISE ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"
SEED = 8101  # Existing Phase 8A-1 TEST-ONLY fixture; not a flight parameter.
SIGMA = np.array([.001, .002, .003])  # Discrete per-sample sigma in S=B, rad/s.


def canonical(value):
    """Exact comparison, retaining sensor metadata and all recorded floats."""
    return json.dumps(value, default=json_value, sort_keys=True, allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def sequence_evidence(ideal, noisy):
    clean_inputs, inputs = (f.attrs["shadow_input_trace"] for f in (ideal, noisy))
    clean = np.array([b.gyro_point_B for b in clean_inputs])
    delivered = np.array([b.gyro_point_B for b in inputs])
    logged = np.array([b.gyro_sample.noise_S_rad_s for b in inputs])
    # Independent generator, no GyroModel calls and no runtime noise readback
    # used to construct the expectation. Sample zero consumes the first draw.
    expected = np.random.Generator(np.random.PCG64(SEED)).standard_normal((len(inputs), 3))*SIGMA
    perturbation = delivered-clean
    tolerance = 2*np.finfo(float).eps*max(1., float(np.max(np.abs(clean))))
    config = profile_config("TEST_NOISE_ONLY")
    checks = {
        "existing_noise_only_configuration": config.seed.value == SEED
            and np.array_equal(config.noise_sigma_S.value, SIGMA)
            and not np.any(config.bias_S.value) and np.array_equal(config.scale.value, np.ones(3))
            and not np.any(config.cross_axis.value) and np.array_equal(config.mounting_C_SB.value, np.eye(3))
            and config.range_S.value is None and config.sample_period_ns.value == 100_000_000
            and config.sample_offset_ns.value == config.latency_ns.value == 0,
        "ideal_measurements_are_truth": np.array_equal(clean, vector(ideal, "omega_B", "rad_s")),
        "exact_seeded_draws_without_extra_consumption": np.array_equal(logged, expected),
        "noise_applied_exactly_once_in_S_and_B": np.array_equal(delivered, clean+expected)
            and all(b.gyro_sample.valid and np.array_equal(b.gyro_sample.measurement_S_rad_s, rate)
                and np.array_equal(b.gyro_sample.measurement_B_rad_s, rate)
                and b.gyro_sample.configuration_fingerprint == config.fingerprint()
                for b, rate in zip(inputs, delivered)),
        "measured_minus_ideal_matches_expected_to_roundoff": np.max(np.abs(perturbation-expected)) <= tolerance,
        "all_original_sample_epochs_preserved": len(inputs) == len(clean_inputs) == 31
            and all(b.gyro_epoch_ns == c.gyro_epoch_ns == b.sim_epoch_ns == c.sim_epoch_ns
                == b.gyro_sample.acquisition_ns == b.gyro_sample.publication_ns == j*100_000_000
                for j, (b, c) in enumerate(zip(inputs, clean_inputs))),
    }
    return dict(checks={k:bool(v) for k,v in checks.items()}, expected_noise_B_rad_s=expected.tolist(),
        measured_minus_ideal_B_rad_s=perturbation.tolist(), logged_noise_S_rad_s=logged.tolist(),
        max_subtraction_roundoff_rad_s=float(np.max(np.abs(perturbation-expected))),
        subtraction_tolerance_rad_s=tolerance, delivered_sha256=digest(delivered), expected_noise_sha256=digest(expected))


def extra_response(case):
    history = [h for h in case["history"] if "estimated_bias_B_rad_s" in h]
    estimates = np.array([h["estimated_bias_B_rad_s"] for h in history])
    all_estimates = np.concatenate([estimates, np.array([e[stage]["bias_B_rad_s"]
        for e in case["raw_observe_callbacks"] if e["initialized_before"] and e["result"] == "updated"
        for stage in ("before", "after")])])
    rotations = np.array([h["excess_rotation_B_rad"] for h in history])
    update_ticks = {e["processing_ns"] for e in case["all_update_callbacks"]}
    variations = [dict(start_ns=a["processing_ns"], end_ns=b["processing_ns"],
        vector_update_processed=b["processing_ns"] in update_ticks,
        error_vector_change_B_rad=change.tolist(), change_norm_rad=float(np.linalg.norm(change)))
        for a,b,change in zip(history, history[1:], np.diff(rotations, axis=0))]
    holds = [np.array_equal(a["estimated_bias_B_rad_s"], b["estimated_bias_B_rad_s"])
        for a,b in zip(history, history[1:]) if b["processing_ns"] not in update_ticks]
    return dict(max_bias_estimate_norm_rad_s=float(np.max(np.linalg.norm(all_estimates, axis=1))),
        min_bias_by_axis_rad_s=all_estimates.min(axis=0).tolist(), max_bias_by_axis_rad_s=all_estimates.max(axis=0).tolist(),
        sample_to_sample_error_changes=variations,
        max_gyro_only_error_change_rad=max(v["change_norm_rad"] for v in variations if not v["vector_update_processed"]),
        bias_held_during_gyro_only_intervals=bool(holds) and all(holds))


def short_interval(ideal_case, noise_case, sequence, noisy):
    # First propagation after the identical vector acquisition, no bias update.
    # The existing linear endpoint reconstruction integrates to the trapezoid.
    samples = np.array(sequence["measured_minus_ideal_B_rad_s"])[4:6]
    expected = .5*(samples[0]+samples[1])*.1
    i, n = ideal_case["history"][5], noise_case["history"][5]
    observed = np.array(n["excess_rotation_B_rad"])-i["excess_rotation_B_rad"]
    peak_noise = float(np.max(np.linalg.norm(samples, axis=1)))
    peak_rate = float(np.max(np.linalg.norm(vector(noisy, "omega_B", "rad_s")[4:6], axis=1)))
    # First neglected body-rotation transport, using endpoint peak noise/rate.
    # A local first-order allowance, not stochastic accuracy or long-term ARW.
    allowance = .5*(peak_rate+peak_noise)*peak_noise*.1**2 + 1e-10
    context_ok = np.array_equal(ideal_case["history"][4]["estimated_q_BN"], noise_case["history"][4]["estimated_q_BN"])
    context_ok = context_ok and np.array_equal(n["estimated_bias_B_rad_s"], np.zeros(3))
    context_ok = context_ok and not any(e["processing_ns"] == 500_000_000 for e in noise_case["all_update_callbacks"])
    return dict(start_ns=400_000_000, end_ns=500_000_000, endpoint_noise_B_rad_s=samples.tolist(),
        expected_trapezoidal_rotation_B_rad=expected.tolist(), observed_excess_rotation_B_rad=observed.tolist(),
        expected_norm_rad=float(np.linalg.norm(expected)), observed_norm_rad=float(np.linalg.norm(observed)),
        discrepancy_rad=float(np.linalg.norm(observed-expected)), transport_allowance_rad=allowance,
        dot=float(observed @ expected), context_verified=bool(context_ok),
        passed=bool(context_ok and observed @ expected > 0 and np.linalg.norm(observed-expected) <= allowance))


def repeatability(first_frame, second_frame, first_case, second_case):
    return {
        "repeat_all_input_samples_intervals_events_exact": canonical([asdict(b) for b in first_frame.attrs["shadow_input_trace"]])
            == canonical([asdict(b) for b in second_frame.attrs["shadow_input_trace"]]),
        "repeat_estimator_states_covariance_counts_exact": canonical(first_frame.attrs["shadow_estimator_trace"])
            == canonical(second_frame.attrs["shadow_estimator_trace"]),
        "repeat_innovations_witnesses_and_status_exact": canonical(first_case) == canonical(second_case),
        "repeat_native_navigation_telemetry_exact": csv_bytes(first_frame.attrs["shadow_telemetry"])
            == csv_bytes(second_frame.attrs["shadow_telemetry"]),
    }


def preserve_ideal(frame, case):
    path = HERE/"output_data/phase8a2a_gyro_bias.json"
    prior = json.loads(path.read_text(encoding="utf-8")) if path.exists() else None
    if prior is not None and all(hashlib.sha256((HERE/n).read_bytes()).hexdigest() == h for n,h in prior["source_sha256"].items()):
        return dict(method="source-bound saved 8A-2A ideal reference", passed=bool(prior["passed"]
            and canonical(case) == canonical(prior["evidence"]["ideal"])
            and hashlib.sha256(csv_bytes(frame)).hexdigest() == prior["host_sha256"]["ideal"]),
            artifact_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    # Portable fallback if ignored evidence is absent. Shared source must still
    # match HEAD; compare against an uninstrumented run of the same fixture.
    with contextlib.redirect_stdout(io.StringIO()):
        plain = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            shadow=ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000, gyro_model=profile_config()))
    return dict(method="uninstrumented unchanged ideal reference", passed=csv_bytes(plain) == csv_bytes(frame)
        and canonical(plain.attrs["shadow_estimator_trace"]) == canonical(frame.attrs["shadow_estimator_trace"]))


def validate():
    ideal, iw = run_case("IDEAL_REGRESSION")
    noisy, nw = run_case("TEST_NOISE_ONLY")
    repeated, rw = run_case("TEST_NOISE_ONLY")
    # Validate stochastic delivery BEFORE interpreting any estimator response.
    sequence = sequence_evidence(ideal, noisy)
    ideal_case, noise_case, repeat_case = (characterize(f, w, np.zeros(3))
        for f,w in ((ideal,iw), (noisy,nw), (repeated,rw)))
    preservation = preserve_ideal(ideal, ideal_case)
    short = short_interval(ideal_case, noise_case, sequence, noisy)
    response = {name:extra_response(case) for name,case in (("ideal",ideal_case),("noise",noise_case))}
    checks = sequence["checks"] | repeatability(noisy, repeated, noise_case, repeat_case)
    stats, base = noise_case["summary"], ideal_case["summary"]
    valid = [h for h in noise_case["history"] if "principal_error_rad" in h]
    updates = noise_case["all_update_callbacks"]
    policy = json.loads((HERE/"config/attitude_mekf_test_only.json").read_text(encoding="utf-8"))
    p0 = np.array(policy["p0_diagonal"]["value"])
    trace_bound = 2*p0[:3].sum()+2*DURATION_S**2*p0[3:].sum()
    guard = False
    try:
        run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            shadow=ShadowOptions(ideal_sun=True, gyro_model=profile_config("TEST_NOISE_ONLY")), control_source="MEKF_DEVELOPMENT")
    except ValueError as error:
        guard = "MEKF_DEVELOPMENT requires" in str(error)
    runtime = ("gyro_sensor_model.py", "attitude_mekf.py", "attitude_mekf_adapter.py",
        "scenario_huskysat2_detumble.py", "config/attitude_mekf_test_only.json", "validate_gyro_bias_response.py")
    checks.update({
        "finite_initialized_response": stats["valid_after_acquisition"] and stats["final_state"] == "INITIALIZED"
            and all(np.isfinite(h["estimated_q_BN"]).all() and np.isfinite(h["estimated_bias_B_rad_s"]).all()
                and np.isfinite(h["principal_error_rad"]) for h in valid),
        "posterior_bias_subtracted_once": stats["maximum_published_rate_contract_error_rad_s"] == 0.,
        "bias_changes_only_on_vector_processing": response["noise"]["bias_held_during_gyro_only_intervals"],
        "random_samples_not_assigned_as_fixed_bias": all(not np.array_equal(h["estimated_bias_B_rad_s"], h["gyro_sample"]["noise_S_rad_s"]) for h in valid),
        "independent_short_interval_direction_and_magnitude": short["passed"],
        "finite_geometrically_consistent_innovations": bool(updates) and all(np.isfinite(u["tangent_innovation"]).all()
            and abs(u["pre_residual_rad"]-u["recorded_residual_rad"]) <= ROUND_OFF
            and abs(np.linalg.norm(u["tangent_innovation"])-u["independent_tangent_norm"]) <= ROUND_OFF
            and u["descent_dot"] >= -ROUND_OFF**2 for u in updates),
        "actual_updates_reduce_measured_vector_residuals": bool(updates) and all(u["post_residual_rad"] <= u["pre_residual_rad"]+ROUND_OFF for u in updates),
        "both_vector_residual_histories_perturbed": all(
            [u["pre_residual_rad"] for u in noise_case["unique_updates"] if u["sensor"] == kind]
            != [u["pre_residual_rad"] for u in ideal_case["unique_updates"] if u["sensor"] == kind]
            for kind in ("magnetic", "sun")),
        "covariance_finite_symmetric_positive_definite": all(
            c["summary"]["covariance_max_asymmetry"] <= c["summary"]["covariance_numerical_tolerance"]
            and c["summary"]["covariance_min_eigenvalue"] > c["summary"]["covariance_numerical_tolerance"]
            and np.isfinite(c["summary"]["covariance_max_eigenvalue"])
            and all(np.isfinite(h["covariance"]).all() and np.all(np.array(h["covariance_diagonal"]) > 0)
                for h in c["history"] if "covariance" in h) for c in (ideal_case,noise_case)),
        "no_covariance_growth_beyond_zero_Q_kinematic_bound": not np.any(policy["qc_diagonal"]["value"]) and all(
            np.all(np.array(c["summary"]["maximum_bias_covariance_diagonal"]) <= p0[3:]+ROUND_OFF)
            and c["summary"]["maximum_attitude_covariance_trace"] <= trace_bound+ROUND_OFF for c in (ideal_case,noise_case)),
        "same_vector_events_replays_and_no_rejections": stats["update_counts"] == base["update_counts"]
            and stats["replay_count"] == base["replay_count"] and stats["rejected_event_count"] == base["rejected_event_count"] == 0
            and [u["event_id"] for u in noise_case["unique_updates"]] == [u["event_id"] for u in ideal_case["unique_updates"]],
        "host_spacecraft_commands_torques_byte_identical": csv_bytes(ideal) == csv_bytes(noisy) == csv_bytes(repeated),
        # run_case explicitly selects SIMPLE_NAV_REFERENCE. The unchanged
        # scenario wires this branch to nav.attOutMsg; only the MEKF owner branch
        # exports control_source metadata. Do not invent default metadata.
        "SimpleNav_control_without_development_command_owner": all("control_source" not in f.attrs
            and "control_source" not in f.columns and "mekf_command_owner" not in f.attrs for f in (ideal,noisy,repeated)),
        "modeled_gyro_closed_loop_rejected": guard,
        "ideal_reference_preserved": preservation["passed"],
        "shared_runtime_and_Q_R_P0_unchanged": subprocess.run(["git", "diff", "--quiet", "HEAD", "--",
            *("basilisk_runner/"+n for n in runtime)], cwd=ROOT).returncode == 0,
    })
    sources = runtime + ("validate_gyro_noise_response.py", "test_gyro_noise_response.py", "validate_mekf_closed_loop.py",
        "validate_gyro_sensor_model.py", "magnetic_control_cycle.py", "hs2_sim_config.py")
    report = dict(scope=SCOPE, passed=all(checks.values()), checks={k:bool(v) for k,v in checks.items()},
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources}, numpy_version=np.__version__,
        duration_s=DURATION_S, sequence=sequence, reference_preservation=preservation,
        configuration=ideal.attrs["simulation_config"], options=ideal.attrs["shadow_options"], estimator_policy=policy,
        gyro_profiles={p:profile_config(p).to_dict() for p in ("IDEAL_REGRESSION","TEST_NOISE_ONLY")},
        covariance_attitude_trace_bound_rad2=float(trace_bound), independent_short_interval=short,
        ideal=ideal_case, noise=noise_case, response=response,
        repeat_case_sha256=digest(repeat_case), first_noise_case_sha256=digest(noise_case),
        host_sha256={name:hashlib.sha256(csv_bytes(f)).hexdigest() for name,f in (("ideal",ideal),("noise",noisy),("repeat",repeated))})
    return report, ideal, noisy, repeated, noise_case, repeat_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase8a2b_gyro_noise.json")
    args = parser.parse_args()
    report, ideal, noisy, repeated, _, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, default=json_value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for name,frame in (("ideal",ideal),("noise",noisy),("repeat",repeated)):
        args.report.with_name(args.report.stem+"_"+name+".csv").write_bytes(csv_bytes(frame))
    print(json.dumps(dict(passed=report["passed"], checks=report["checks"], reference_preservation=report["reference_preservation"],
        ideal=report["ideal"]["summary"], noise=report["noise"]["summary"], short_interval=report["independent_short_interval"],
        response={k:{n:v for n,v in r.items() if n != "sample_to_sample_error_changes"} for k,r in report["response"].items()}),
        default=json_value, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
