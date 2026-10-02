"""8A-2C live bias + noise composability, ASSUMED / TEST-ONLY, 2026-10-02.

Construct the sole combined profile here, without extending flight configuration
or the runtime gyro-profile registry. Reuse source-bound 8A-2A/2B evidence and
their read-only diagnostics. Only missing/unbound references are rerun.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import replace
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
    DURATION_S, HERE, ROOT, ROUND_OFF, INJECTED_BIAS, LiveUpdateWitness,
    characterize, json_value, run_case,
)
from validate_gyro_noise_response import SEED, SIGMA, canonical, digest, extra_response, repeatability
from validate_mekf_closed_loop import csv_bytes, vector

SCOPE = "COMBINED SYNTHETIC GYRO BIAS + NOISE RESPONSE / SHADOW DEVELOPMENT TEST / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"


def combined_profile():
    """Compose existing sourced Parameters; no new magnitudes or runtime mode."""
    noise = profile_config("TEST_NOISE_ONLY")
    return replace(profile_config("TEST_BIAS_ONLY"), profile="TEST_BIAS_PLUS_NOISE",
        noise_sigma_S=noise.noise_sigma_S, seed=noise.seed)


def run_combined():
    witness = LiveUpdateWitness()
    with witness.record(), contextlib.redirect_stdout(io.StringIO()):
        frame = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), control_source="SIMPLE_NAV_REFERENCE",
            shadow=ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000, gyro_model=combined_profile()))
    return frame, witness


def references():
    cases, hosts, provenance = {}, {}, []
    for name, roles in (("phase8a2a_gyro_bias.json", ("ideal", "bias")),
                        ("phase8a2b_gyro_noise.json", ("ideal", "noise"))):
        path = HERE/"output_data"/name
        prior = json.loads(path.read_text(encoding="utf-8")) if path.exists() else None
        bound = prior is not None and prior["passed"] and all(
            hashlib.sha256((HERE/n).read_bytes()).hexdigest() == h for n,h in prior["source_sha256"].items())
        if not bound:
            provenance.append(dict(path=str(path.relative_to(ROOT)), reused=False, reason="missing or not source-bound passed evidence"))
            continue
        provenance.append(dict(path=str(path.relative_to(ROOT)), reused=True,
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(), source_sha256=prior["source_sha256"], base_commit=prior["base_commit"]))
        for role in roles:
            case = prior["evidence"][role] if "evidence" in prior else prior[role]
            host = prior["host_sha256"][role]
            if role in cases and (canonical(cases[role]) != canonical(case) or hosts[role] != host):
                raise ValueError("Conflicting source-bound ideal references; investigate before interpretation")
            cases[role], hosts[role] = case, host
    for role, profile in (("ideal", "IDEAL_REGRESSION"), ("bias", "TEST_BIAS_ONLY"), ("noise", "TEST_NOISE_ONLY")):
        if role not in cases:
            frame, witness = run_case(profile)
            cases[role] = characterize(frame, witness, INJECTED_BIAS if role == "bias" else np.zeros(3))
            hosts[role] = hashlib.sha256(csv_bytes(frame)).hexdigest()
            provenance.append(dict(role=role, reused=False, method="short unchanged live reference regenerated"))
    return cases, hosts, provenance


def composition(frame, refs):
    inputs = frame.attrs["shadow_input_trace"]
    config = combined_profile()
    clean = np.array([h["gyro_delivered_B_rad_s"] for h in refs["ideal"]["history"]])
    noisy = np.array([h["gyro_delivered_B_rad_s"] for h in refs["noise"]["history"]])
    noise_only_draws = np.array([h["gyro_sample"]["noise_S_rad_s"] for h in refs["noise"]["history"]])
    # Independent expectation, never constructed from model readback.
    expected_noise = np.random.Generator(np.random.PCG64(SEED)).standard_normal((len(inputs), 3))*SIGMA
    delivered = np.array([b.gyro_point_B for b in inputs])
    raw_noise = np.array([b.gyro_sample.noise_S_rad_s for b in inputs])
    expected = clean+INJECTED_BIAS+expected_noise
    tolerance = 2*np.finfo(float).eps*max(1., float(np.max(np.abs(clean))))
    checks = {
        "only_authorized_combined_TEST_ONLY_profile": config.profile == "TEST_BIAS_PLUS_NOISE"
            and config.runtime_usable_for_flight is False and np.array_equal(config.bias_S.value, [.003,-.002,.001])
            and np.array_equal(config.noise_sigma_S.value, [.001,.002,.003]) and config.seed.value == 8101
            and np.array_equal(config.mounting_C_SB.value, np.eye(3)) and np.array_equal(config.scale.value, np.ones(3))
            and not np.any(config.cross_axis.value) and config.range_S.value is None
            and config.sample_period_ns.value == 100_000_000 and config.sample_offset_ns.value == config.latency_ns.value == 0,
        "current_truth_matches_ideal_reference": np.array_equal(vector(frame, "omega_B", "rad_s"), clean),
        "exact_bias_plus_noise_composition_S_and_B": np.array_equal(delivered, expected)
            and all(b.gyro_sample.valid and np.array_equal(b.gyro_sample.measurement_S_rad_s, rate)
                and np.array_equal(b.gyro_sample.measurement_B_rad_s, rate)
                and b.gyro_sample.configuration_fingerprint == config.fingerprint() for b,rate in zip(inputs, expected)),
        "same_independent_seeded_draws_as_noise_only": np.array_equal(raw_noise, expected_noise)
            and np.array_equal(raw_noise, noise_only_draws),
        "deterministic_bias_once_relative_to_same_noise_case": np.max(np.abs(delivered-noisy-INJECTED_BIAS)) <= tolerance,
        "acquisition_publication_processing_epochs_preserved": len(inputs) == len(clean) == len(noisy) == 31
            and all(b.gyro_sample.acquisition_ns == b.gyro_sample.publication_ns == b.sim_epoch_ns == b.gyro_epoch_ns
                == j*100_000_000 == refs["ideal"]["history"][j]["acquisition_ns"]
                == refs["noise"]["history"][j]["acquisition_ns"] for j,b in enumerate(inputs)),
    }
    return dict(checks={k:bool(v) for k,v in checks.items()}, expected_noise_B_rad_s=expected_noise.tolist(),
        delivered_B_rad_s=delivered.tolist(), expected_B_rad_s=expected.tolist(),
        combined_minus_noise_only_B_rad_s=(delivered-noisy).tolist(),
        max_bias_difference_roundoff_rad_s=float(np.max(np.abs(delivered-noisy-INJECTED_BIAS))),
        subtraction_tolerance_rad_s=tolerance, raw_noise_sha256=digest(raw_noise))


def short_interval(case, ideal, sequence, frame):
    noise = np.array(sequence["expected_noise_B_rad_s"])[4:6]
    endpoints = INJECTED_BIAS+noise
    expected = .5*(endpoints[0]+endpoints[1])*.1
    observed = np.array(case["history"][5]["excess_rotation_B_rad"])-ideal["history"][5]["excess_rotation_B_rad"]
    peak = float(np.max(np.linalg.norm(endpoints, axis=1)))
    rate = float(np.max(np.linalg.norm(vector(frame, "omega_B", "rad_s")[4:6], axis=1)))
    allowance = .5*(rate+peak)*peak*.1**2 + 1e-10
    context = np.array_equal(case["history"][4]["estimated_q_BN"], ideal["history"][4]["estimated_q_BN"])
    context = context and np.array_equal(case["history"][5]["estimated_bias_B_rad_s"], np.zeros(3))
    context = context and not any(u["processing_ns"] == 500_000_000 for u in case["all_update_callbacks"])
    return dict(start_ns=400_000_000, end_ns=500_000_000, total_endpoint_perturbation_B_rad_s=endpoints.tolist(),
        expected_trapezoidal_rotation_B_rad=expected.tolist(), observed_excess_rotation_B_rad=observed.tolist(),
        expected_norm_rad=float(np.linalg.norm(expected)), observed_norm_rad=float(np.linalg.norm(observed)),
        discrepancy_rad=float(np.linalg.norm(observed-expected)), transport_allowance_rad=allowance,
        dot=float(observed @ expected), context_verified=bool(context),
        passed=bool(context and observed @ expected > 0 and np.linalg.norm(observed-expected) <= allowance))


def validate():
    refs, reference_hosts, provenance = references()
    frame, witness = run_combined()
    repeated, repeat_witness = run_combined()
    sequence = composition(frame, refs)
    checks = sequence["checks"].copy()
    runtime = ("gyro_sensor_model.py", "attitude_mekf.py", "attitude_mekf_adapter.py", "scenario_huskysat2_detumble.py",
        "config/attitude_mekf_test_only.json", "validate_gyro_bias_response.py", "validate_gyro_noise_response.py",
        "validate_mekf_closed_loop.py", "magnetic_control_cycle.py", "hs2_sim_config.py")
    sources = runtime + ("validate_gyro_bias_noise_response.py", "test_gyro_bias_noise_response.py")
    report = dict(scope=SCOPE, passed=False, checks=checks, composition=sequence, references=provenance,
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources}, numpy_version=np.__version__,
        duration_s=DURATION_S, combined_profile=combined_profile().to_dict(),
        profile_status="ASSUMED / TEST-ONLY", profile_fingerprint=combined_profile().fingerprint(),
        configuration=frame.attrs["simulation_config"], options=frame.attrs["shadow_options"],
        host_sha256={"combined":hashlib.sha256(csv_bytes(frame)).hexdigest(), "repeat":hashlib.sha256(csv_bytes(repeated)).hexdigest()},
        reference_host_sha256=reference_hosts)
    # Critical measurement gate: preserve evidence and fail without interpreting
    # the filter if composition is incorrect. CLI still saves both host CSVs.
    if not all(checks.values()):
        report["blocked_interpretation"] = "Measurement composition failed before estimator analysis"
        return report, frame, repeated
    case = characterize(frame, witness, INJECTED_BIAS)
    repeat_case = characterize(repeated, repeat_witness, INJECTED_BIAS)
    cases = refs | {"combined":case}
    response = {name:extra_response(c) for name,c in cases.items()}
    short = short_interval(case, refs["ideal"], sequence, frame)
    checks.update(repeatability(frame, repeated, case, repeat_case))
    stats = case["summary"]
    valid = [h for h in case["history"] if "principal_error_rad" in h]
    last_update_at_tick = {u["processing_ns"]:u for u in case["all_update_callbacks"]}
    bias_increment = np.array(stats["final_bias_B_rad_s"])-refs["noise"]["summary"]["final_bias_B_rad_s"]
    policy = json.loads((HERE/"config/attitude_mekf_test_only.json").read_text(encoding="utf-8"))
    p0 = np.array(policy["p0_diagonal"]["value"])
    trace_bound = 2*p0[:3].sum()+2*DURATION_S**2*p0[3:].sum()
    guard = False
    try:
        run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            shadow=ShadowOptions(ideal_sun=True, gyro_model=combined_profile()), control_source="MEKF_DEVELOPMENT")
    except ValueError as error:
        guard = "MEKF_DEVELOPMENT requires" in str(error)
    checks.update({
        "finite_valid_response": stats["valid_after_acquisition"] and stats["final_state"] == "INITIALIZED"
            and all(np.isfinite(h["estimated_q_BN"]).all() and np.isfinite(h["estimated_bias_B_rad_s"]).all()
                and np.isfinite(h["principal_error_rad"]) for h in valid),
        # Directional evidence only: no convergence speed, superposition or
        # monotonic-sign requirement on individual noisy update corrections.
        "final_bias_and_matched_noise_difference_have_injected_sign": np.all(np.array(stats["final_bias_B_rad_s"])*INJECTED_BIAS > 0)
            and np.all(bias_increment*INJECTED_BIAS > 0),
        "bias_held_in_propagation_and_explained_by_vector_updates": response["combined"]["bias_held_during_gyro_only_intervals"]
            and all(np.array_equal(h["estimated_bias_B_rad_s"], last_update_at_tick[h["processing_ns"]]["estimated_bias_B_rad_s"])
                for h in valid if h["processing_ns"] in last_update_at_tick),
        "posterior_bias_subtracted_once": stats["maximum_published_rate_contract_error_rad_s"] == 0.,
        "independent_endpoint_interpolated_short_interval": short["passed"],
        "finite_consistent_vector_innovation_direction": bool(case["all_update_callbacks"]) and all(
            np.isfinite(u["tangent_innovation"]).all() and abs(u["pre_residual_rad"]-u["recorded_residual_rad"]) <= ROUND_OFF
            and abs(np.linalg.norm(u["tangent_innovation"])-u["independent_tangent_norm"]) <= ROUND_OFF
            and u["descent_dot"] >= -ROUND_OFF**2 for u in case["all_update_callbacks"]),
        "every_actual_vector_update_reduces_measured_residual": all(u["post_residual_rad"] <= u["pre_residual_rad"]+ROUND_OFF
            for u in case["all_update_callbacks"]),
        "both_vector_histories_expose_combined_errors": all(
            [u["pre_residual_rad"] for u in case["unique_updates"] if u["sensor"] == kind]
            != [u["pre_residual_rad"] for u in refs["ideal"]["unique_updates"] if u["sensor"] == kind]
            for kind in ("magnetic", "sun")),
        "covariance_finite_symmetric_positive_definite": stats["covariance_max_asymmetry"] <= stats["covariance_numerical_tolerance"]
            and stats["covariance_min_eigenvalue"] > stats["covariance_numerical_tolerance"] and np.isfinite(stats["covariance_max_eigenvalue"])
            and all(np.isfinite(h["covariance"]).all() and np.all(np.array(h["covariance_diagonal"]) > 0) for h in valid),
        "no_covariance_growth_beyond_zero_Q_kinematic_bound": not np.any(policy["qc_diagonal"]["value"])
            and np.all(np.array(stats["maximum_bias_covariance_diagonal"]) <= p0[3:]+ROUND_OFF)
            and stats["maximum_attitude_covariance_trace"] <= trace_bound+ROUND_OFF,
        "same_vector_events_replays_and_no_rejections": stats["rejected_event_count"] == 0
            and all(stats["update_counts"] == c["summary"]["update_counts"] and stats["replay_count"] == c["summary"]["replay_count"]
                and c["summary"]["rejected_event_count"] == 0
                and [u["event_id"] for u in case["unique_updates"]] == [u["event_id"] for u in c["unique_updates"]] for c in refs.values()),
        "host_spacecraft_actuators_match_all_references_and_repeat": csv_bytes(frame) == csv_bytes(repeated)
            and all(h == report["host_sha256"]["combined"] for h in reference_hosts.values()),
        # Explicit SIMPLE_NAV_REFERENCE in run_combined; unchanged scenario
        # subscribes the controller/cycle to SimpleNav's nav.attOutMsg. Only the
        # MEKF owner branch emits control_source metadata. No default invented.
        "SimpleNav_control_without_development_command_owner": all("control_source" not in f.attrs
            and "control_source" not in f.columns and "mekf_command_owner" not in f.attrs for f in (frame,repeated)),
        "combined_modeled_gyro_closed_loop_rejected": guard,
        "shared_runtime_helpers_and_Q_R_P0_unchanged": subprocess.run(["git", "diff", "--quiet", "HEAD", "--",
            *("basilisk_runner/"+n for n in runtime)], cwd=ROOT).returncode == 0,
    })
    report.update(passed=all(checks.values()), checks={k:bool(v) for k,v in checks.items()}, cases=cases, response=response,
        independent_short_interval=short, estimator_policy=policy, covariance_attitude_trace_bound_rad2=float(trace_bound),
        final_bias_increment_over_noise_only_B_rad_s=bias_increment.tolist(),
        combined_case_sha256=digest(case), repeat_case_sha256=digest(repeat_case))
    return report, frame, repeated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase8a2c_gyro_bias_noise.json")
    args = parser.parse_args()
    report, frame, repeated = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, default=json_value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for name,f in (("combined",frame),("repeat",repeated)):
        args.report.with_name(args.report.stem+"_"+name+".csv").write_bytes(csv_bytes(f))
    print(json.dumps(dict(passed=report["passed"], checks=report["checks"],
        blocked_interpretation=report.get("blocked_interpretation"),
        references=[{k:v for k,v in p.items() if k != "source_sha256"} for p in report["references"]],
        summaries={name:c["summary"] | {k:v for k,v in report["response"][name].items() if k != "sample_to_sample_error_changes"}
            for name,c in report.get("cases",{}).items()}, short_interval=report.get("independent_short_interval")),
        default=json_value, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
