"""8A-2A deterministic live gyro-bias response, TEST-ONLY, 2026-10-01.

Reuse exactly the 8A-1 three-second ideal/bias profiles and vector schedule.
Temporary validation wrappers observe existing calls without changing arguments,
returns, state, Q/R/P0 or equations. Every replay callback is retained separately
from unique final-history vector events. No actuator endpoint is obtained.
"""
from __future__ import annotations

import argparse
import contextlib
from copy import deepcopy
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import subprocess
from unittest.mock import patch

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import MEKF
from attitude_mekf_adapter import MEKFNavigationAdapter, ShadowOptions
from gyro_sensor_model import profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_gyro_sensor_model import normalized
from validate_mekf_closed_loop import csv_bytes, mrp, vector

HERE, ROOT = Path(__file__).resolve().parent, Path(__file__).resolve().parents[1]
DURATION_S = 3.0  # Existing 8A-1 horizon, not convergence/flight acceptance.
INJECTED_BIAS = np.array([.003, -.002, .001])  # User-specified synthetic S=B case, rad/s.
ROUND_OFF = 256*np.finfo(float).eps  # Numerical diagnostic, not innovation acceptance.
SCOPE = "SYNTHETIC GYRO-BIAS ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"


def json_value(value):
    # Preserve gyro_sample metadata; the older equivalence normalizer deliberately
    # excludes that additive field and is unsuitable for evidence serialization.
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Unsupported evidence type: {type(value).__name__}")


def angle_between(a, b):
    return float(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b)))


def passive_rotation(c):
    """Principal rotation vector for a passive DCM; independent of MEKF helpers.

    For C_est C_truth.T this is excess estimated rotation in B, approximately
    +b*dt for an uncorrected positive gyro bias. It is the negative of the
    MEKF's small true-minus-estimate correction convention.
    """
    sine_axis = .5*np.array([c[1, 2]-c[2, 1], c[2, 0]-c[0, 2], c[0, 1]-c[1, 0]])
    sine = np.linalg.norm(sine_axis)
    angle = float(np.arctan2(sine, (np.trace(c)-1)/2))
    if angle >= np.pi/2:
        raise ValueError("development error escaped the small-angle diagnostic domain")
    return sine_axis if sine < 1e-15 else sine_axis*(angle/sine)


class LiveUpdateWitness:
    """Read-only observation of actual core updates, including internal replay."""
    def __init__(self):
        self.processing_ns = None
        self.events: list[dict] = []

    @contextlib.contextmanager
    def record(self):
        original_update, original_observe = MEKFNavigationAdapter.UpdateState, MEKF.observe

        def update(adapter, tick):
            self.processing_ns = int(tick)
            return original_update(adapter, tick)

        def observe(core, sample):
            before = deepcopy(core.output())
            initialized = core.initialized
            result = original_observe(core, sample)
            self.events.append(dict(callback_index=len(self.events), processing_ns=self.processing_ns,
                acquisition_ns=sample.epoch_ns, event_id=sample.event_id, sensor=sample.sensor,
                sample=asdict(sample), initialized_before=initialized, result=result,
                before=before, after=deepcopy(core.output())))
            return result

        with patch.object(MEKFNavigationAdapter, "UpdateState", update), patch.object(MEKF, "observe", observe):
            yield


def run_case(profile):
    witness = LiveUpdateWitness()
    with witness.record(), contextlib.redirect_stdout(io.StringIO()):
        frame = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), control_source="SIMPLE_NAV_REFERENCE",
            shadow=ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000, gyro_model=profile_config(profile)))
    return frame, witness


def characterize(frame, witness, injected):
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    truth_rates, truth_sigma = vector(frame, "omega_B", "rad_s"), mrp(frame)
    truth = {int(t): np.asarray(rbk.MRP2C(s)) for t, s in zip(ticks, truth_sigma)}
    trace, inputs, statuses = (frame.attrs[k] for k in ("shadow_estimator_trace", "shadow_input_trace", "shadow_status"))
    nav = frame.attrs["shadow_telemetry"]
    history = []
    covariances = []
    rate_errors = []
    for i, (row, batch, status) in enumerate(zip(trace, inputs, statuses)):
        core = row["core"]
        item = dict(processing_ns=int(ticks[i]), acquisition_ns=batch.gyro_epoch_ns,
            gyro_sample=asdict(batch.gyro_sample), gyro_delivered_B_rad_s=batch.gyro_point_B.tolist(),
            true_rate_B_rad_s=truth_rates[i].tolist(), status=status)
        if core is not None:
            c = np.asarray(rbk.EP2C(core["q_BN"]))
            error = passive_rotation(c @ truth[int(ticks[i])].T)
            p = np.asarray(core["P"])
            eig = np.linalg.eigvalsh(p)
            covariances.append(p)
            published_rate = nav.iloc[i][[f"mekf_omega_{j}_rad_s" for j in range(3)]].to_numpy(dtype=float)
            rate_errors.append(float(np.max(np.abs(published_rate-(batch.gyro_point_B-core["bias_B_rad_s"])))))
            item.update(estimated_q_BN=np.asarray(core["q_BN"]).tolist(),
                estimated_bias_B_rad_s=np.asarray(core["bias_B_rad_s"]).tolist(),
                excess_rotation_B_rad=error.tolist(), principal_error_rad=float(np.linalg.norm(error)),
                covariance_diagonal=np.diag(p).tolist(), covariance_eigenvalues=eig.tolist(),
                published_rate_B_rad_s=published_rate.tolist(), covariance=p.tolist())
        history.append(item)
    updates = []
    for event in witness.events:
        if not event["initialized_before"] or event["result"] != "updated":
            continue
        sample, pre, post = event["sample"], event["before"], event["after"]
        body = np.array(sample["measured"], dtype=float)
        if sample["frame"] == "S":
            body = np.array(sample["c_sb"]).T @ body
        body /= np.linalg.norm(body)
        reference = np.array(sample["reference_n"], dtype=float)
        reference /= np.linalg.norm(reference)
        pre_c, post_c = np.asarray(rbk.EP2C(pre["q_BN"])), np.asarray(rbk.EP2C(post["q_BN"]))
        predicted = pre_c @ reference
        actual = post["last_update"]
        correction = passive_rotation(post_c @ pre_c.T)
        geometric_descent = np.cross(body, predicted)
        pre_error = passive_rotation(pre_c @ truth[event["acquisition_ns"]].T)
        post_error = passive_rotation(post_c @ truth[event["acquisition_ns"]].T)
        updates.append(dict(callback_index=event["callback_index"], processing_ns=event["processing_ns"],
            acquisition_ns=event["acquisition_ns"], event_id=event["event_id"], sensor=event["sensor"],
            pre_residual_rad=angle_between(body, predicted), post_residual_rad=angle_between(body, post_c @ reference),
            recorded_residual_rad=actual["residual_angle_rad"], tangent_innovation=actual["tangent_innovation"],
            independent_tangent_norm=float(np.linalg.norm(np.cross(body, predicted))),
            geometric_descent_B=geometric_descent.tolist(), correction_B_rad=correction.tolist(),
            descent_dot=float(np.dot(correction, geometric_descent)),
            pre_truth_error_rad=float(np.linalg.norm(pre_error)), post_truth_error_rad=float(np.linalg.norm(post_error)),
            delta_bias_B_rad_s=(np.asarray(post["bias_B_rad_s"])-pre["bias_B_rad_s"]).tolist(),
            estimated_bias_B_rad_s=np.asarray(post["bias_B_rad_s"]).tolist()))
        covariances.extend((np.asarray(pre["P"]), np.asarray(post["P"])))
    # A replay can evaluate a previously received event again. Keep every callback
    # above; select its LAST occurrence for the final retained-history summary.
    unique = sorted({u["event_id"]: u for u in updates}.values(), key=lambda u: u["acquisition_ns"])
    valid = [h for h in history if "principal_error_rad" in h]
    eigenvalues = np.linalg.eigvalsh(np.array(covariances))
    covariance_scale = max(1., float(np.max(np.linalg.norm(covariances, axis=(1, 2)))))
    symmetry = float(np.max(np.abs(np.array(covariances)-np.array(covariances).transpose(0, 2, 1))))
    bias_final = np.array(valid[-1]["estimated_bias_B_rad_s"])
    first = []
    for axis in range(3):
        event = next((u for u in updates if abs(u["delta_bias_B_rad_s"][axis]) > ROUND_OFF), None)
        first.append(None if event is None else dict(processing_ns=event["processing_ns"], acquisition_ns=event["acquisition_ns"],
            sensor=event["sensor"], delta_rad_s=event["delta_bias_B_rad_s"][axis]))
    summary = dict(initial_bias_B_rad_s=valid[0]["estimated_bias_B_rad_s"], first_bias_correction=first,
        final_bias_B_rad_s=bias_final.tolist(), final_bias_error_B_rad_s=(bias_final-injected).tolist(),
        final_attitude_error_rad=valid[-1]["principal_error_rad"],
        max_published_attitude_error_rad=max(h["principal_error_rad"] for h in valid),
        max_witnessed_attitude_error_rad=max([h["principal_error_rad"] for h in valid]
            + [u[key] for u in updates for key in ("pre_truth_error_rad", "post_truth_error_rad")]),
        max_gyro_only_published_error_rad=max(h["principal_error_rad"] for h in valid
            if h["processing_ns"] not in {u["processing_ns"] for u in updates}),
        final_excess_rotation_B_rad=valid[-1]["excess_rotation_B_rad"],
        maximum_vector_residual_rad={kind:max(u["pre_residual_rad"] for u in unique if u["sensor"] == kind)
            for kind in ("magnetic", "sun")},
        maximum_tangent_innovation={kind:max(float(np.linalg.norm(u["tangent_innovation"])) for u in unique if u["sensor"] == kind)
            for kind in ("magnetic", "sun")},
        covariance_min_eigenvalue=float(eigenvalues.min()), covariance_max_eigenvalue=float(eigenvalues.max()),
        covariance_max_asymmetry=symmetry, covariance_numerical_tolerance=ROUND_OFF*covariance_scale,
        maximum_attitude_covariance_trace=float(np.max(np.trace(np.array(covariances)[:, :3, :3], axis1=1, axis2=2))),
        maximum_bias_covariance_diagonal=np.max(np.diagonal(np.array(covariances)[:, 3:, 3:], axis1=1, axis2=2), axis=0).tolist(),
        final_attitude_covariance_diag=valid[-1]["covariance_diagonal"][:3],
        final_bias_covariance_diag=valid[-1]["covariance_diagonal"][3:],
        initial_state=statuses[0]["state"], final_state=statuses[-1]["state"], valid_after_acquisition=all(s["valid"] for s in statuses[4:]),
        rejected_event_count=sum(statuses[-1]["rejected"].values()), rejected_reasons=statuses[-1]["rejected"],
        update_counts=statuses[-1]["updates"], replay_count=statuses[-1]["replays"],
        actual_update_callbacks=len(updates), unique_updated_vector_events=len(unique),
        maximum_published_rate_contract_error_rad_s=max(rate_errors),
        updates_increasing_full_attitude_error=sum(u["post_truth_error_rad"] > u["pre_truth_error_rad"]+ROUND_OFF for u in unique))
    return dict(summary=summary, history=history, unique_updates=unique, all_update_callbacks=updates,
                raw_observe_callbacks=normalized(witness.events))


def assess(ideal_frame, bias_frame, ideal_witness, bias_witness):
    ideal = characterize(ideal_frame, ideal_witness, np.zeros(3))
    bias = characterize(bias_frame, bias_witness, INJECTED_BIAS)
    stats = bias["summary"]
    truth_rates = vector(bias_frame, "omega_B", "rad_s")
    inputs = bias_frame.attrs["shadow_input_trace"]
    # First interval following acquisition, with no vector update or bias correction.
    i = next(h for h in ideal["history"] if h["processing_ns"] == 500_000_000)
    b = next(h for h in bias["history"] if h["processing_ns"] == 500_000_000)
    excess = np.array(b["excess_rotation_B_rad"])-i["excess_rotation_B_rad"]
    dt = .1
    expected = INJECTED_BIAS*dt
    # Rotation transports a fixed body-axis rate bias: the first neglected term
    # is bounded by integral |omega x b| t dt. No convergence-speed criterion.
    turn_rate = float(np.max(np.linalg.norm(truth_rates[4:6], axis=1)))+float(np.linalg.norm(INJECTED_BIAS))
    bound = .5*turn_rate*np.linalg.norm(INJECTED_BIAS)*dt**2 + 1e-10
    short = dict(start_ns=400_000_000, end_ns=500_000_000, excess_rotation_B_rad=excess.tolist(),
        expected_first_order_B_rad=expected.tolist(), actual_norm_rad=float(np.linalg.norm(excess)),
        expected_norm_rad=float(np.linalg.norm(expected)), discrepancy_rad=float(np.linalg.norm(excess-expected)),
        transport_remainder_bound_rad=float(bound), dot=float(excess @ expected))
    checks = {
        "bias_applied_once_at_all_live_samples": all(np.array_equal(x.gyro_point_B, rate+INJECTED_BIAS)
            and x.gyro_sample.valid and np.array_equal(x.gyro_sample.measurement_S_rad_s, x.gyro_point_B)
            for x, rate in zip(inputs, truth_rates)),
        "original_acquisition_publication_epochs": all(x.gyro_epoch_ns == x.sim_epoch_ns
            == x.gyro_sample.acquisition_ns == x.gyro_sample.publication_ns for x in inputs),
        "posterior_bias_subtracted_once_in_native_NavAtt": stats["maximum_published_rate_contract_error_rad_s"] == 0.,
        "finite_valid_bias_estimator": stats["valid_after_acquisition"] and stats["final_state"] == "INITIALIZED"
            and all(np.isfinite(h["estimated_bias_B_rad_s"]).all() and np.isfinite(h["principal_error_rad"])
                for h in bias["history"] if "principal_error_rad" in h),
        "bias_state_moves_toward_injected_sign": np.all(np.array(stats["final_bias_B_rad_s"])*INJECTED_BIAS > 0)
            and np.linalg.norm(stats["final_bias_error_B_rad_s"]) < np.linalg.norm(INJECTED_BIAS),
        "first_and_subsequent_bias_estimates_have_consistent_sign": all(e is not None
            and e["delta_rad_s"]*known > 0 for e, known in zip(stats["first_bias_correction"], INJECTED_BIAS))
            and all(np.all(np.array(h["estimated_bias_B_rad_s"])*INJECTED_BIAS >= -ROUND_OFF**2)
                for h in bias["history"] if "estimated_bias_B_rad_s" in h),
        "first_unupdated_interval_matches_independent_bias_tendency": short["dot"] > 0
            and short["discrepancy_rad"] <= bound
            and np.array_equal(b["estimated_bias_B_rad_s"], [0., 0., 0.])
            and not any(e["processing_ns"] == 500_000_000 for e in bias["all_update_callbacks"]),
        "innovation_sign_norm_and_direction": all(np.isfinite(u["tangent_innovation"]).all()
            and abs(u["pre_residual_rad"]-u["recorded_residual_rad"]) <= ROUND_OFF
            and abs(np.linalg.norm(u["tangent_innovation"])-u["independent_tangent_norm"]) <= ROUND_OFF
            and u["descent_dot"] >= -ROUND_OFF**2 for u in bias["all_update_callbacks"]),
        "every_update_reduces_its_measured_vector_residual": all(u["post_residual_rad"] <= u["pre_residual_rad"]+ROUND_OFF
            for u in bias["all_update_callbacks"]),
        "both_vector_types_expose_bias_error": all(stats["maximum_vector_residual_rad"][kind]
            > ideal["summary"]["maximum_vector_residual_rad"][kind] for kind in ("magnetic", "sun")),
        "covariance_numerically_healthy": all(c["summary"]["covariance_max_asymmetry"] <= c["summary"]["covariance_numerical_tolerance"]
            and c["summary"]["covariance_min_eigenvalue"] > c["summary"]["covariance_numerical_tolerance"]
            and np.isfinite(c["summary"]["covariance_max_eigenvalue"])
            for c in (ideal, bias)),
        "same_vector_events_updates_replay_and_no_rejections": stats["update_counts"] == ideal["summary"]["update_counts"]
            and stats["replay_count"] == ideal["summary"]["replay_count"] and stats["rejected_event_count"] == 0
            and [u["event_id"] for u in bias["unique_updates"]] == [u["event_id"] for u in ideal["unique_updates"]],
        "shadow_host_actuation_and_state_byte_identical": csv_bytes(ideal_frame) == csv_bytes(bias_frame)
            and all("mekf_command_owner" not in f.attrs for f in (ideal_frame, bias_frame)),
    }
    return dict(passed=all(checks.values()), checks={k:bool(v) for k,v in checks.items()},
                ideal=ideal, bias=bias, independent_short_interval=short)


def validate():
    ideal, iw = run_case("IDEAL_REGRESSION")
    biased, bw = run_case("TEST_BIAS_ONLY")
    evidence = assess(ideal, biased, iw, bw)
    # Existing committed 8A-1 evidence is a preservation witness, not an expected
    # estimator response manufactured from this run. Check its source binding.
    prior_path = HERE/"output_data/phase8a1_gyro_model.json"
    prior = json.loads(prior_path.read_text(encoding="utf-8")) if prior_path.exists() else None
    bound_prior = prior is not None and all(hashlib.sha256((HERE/n).read_bytes()).hexdigest() == digest
        for n, digest in prior["source_sha256"].items())
    if bound_prior:
        preserved = dict(method="source-bound saved 8A-1 ideal reference", passed=prior["passed"]
            and normalized(ideal.attrs["shadow_estimator_trace"]) == prior["estimator_trace"]
            and hashlib.sha256(csv_bytes(ideal)).hexdigest() == prior["host_sha256"]["ideal_model"])
    else:
        # Portable fallback: run the same current/committed unchanged source
        # without instrumentation. No sensor-model suite or earlier audit rerun.
        with contextlib.redirect_stdout(io.StringIO()):
            plain = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
                shadow=ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000, gyro_model=profile_config()))
        preserved = dict(method="uninstrumented ideal reference", passed=csv_bytes(plain) == csv_bytes(ideal)
            and normalized(plain.attrs["shadow_estimator_trace"]) == normalized(ideal.attrs["shadow_estimator_trace"]))
    guard = False
    try:
        run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            shadow=ShadowOptions(ideal_sun=True, gyro_model=profile_config("TEST_BIAS_ONLY")), control_source="MEKF_DEVELOPMENT")
    except ValueError as error:
        guard = "MEKF_DEVELOPMENT requires" in str(error)
    evidence["checks"]["ideal_reference_preserved"] = bool(preserved["passed"])
    evidence["checks"]["modeled_gyro_closed_loop_guard_effective"] = guard
    runtime = ("gyro_sensor_model.py", "attitude_mekf.py", "attitude_mekf_adapter.py",
               "scenario_huskysat2_detumble.py", "config/attitude_mekf_test_only.json")
    evidence["checks"]["shared_runtime_and_Q_R_P0_unchanged"] = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", *("basilisk_runner/"+n for n in runtime)], cwd=ROOT).returncode == 0
    evidence["checks"]["only_specified_bias_no_noise_or_mount_change"] = all(
        np.array_equal(profile_config(p).mounting_C_SB.value, np.eye(3))
        and np.array_equal(profile_config(p).scale.value, np.ones(3))
        and not np.any(profile_config(p).cross_axis.value)
        and not np.any(profile_config(p).noise_sigma_S.value)
        and profile_config(p).seed.value is None and profile_config(p).range_S.value is None
        for p in ("IDEAL_REGRESSION", "TEST_BIAS_ONLY"))
    policy = json.loads((HERE/"config/attitude_mekf_test_only.json").read_text(encoding="utf-8"))
    p0 = np.array(policy["p0_diagonal"]["value"])
    # With existing Q=0, constant-bias covariance cannot grow in propagation.
    # A loose kinematic bound on attitude trace follows from |integral db dt|
    # <= T|db| and |a+b|^2 <= 2|a|^2+2|b|^2. No statistical calibration claim.
    trace_bound = 2*p0[:3].sum()+2*DURATION_S**2*p0[3:].sum()
    evidence["checks"]["no_covariance_growth_beyond_zero_Q_kinematic_bound"] = not np.any(policy["qc_diagonal"]["value"]) and all(
        np.all(np.array(evidence[c]["summary"]["maximum_bias_covariance_diagonal"]) <= p0[3:]+ROUND_OFF)
        and evidence[c]["summary"]["maximum_attitude_covariance_trace"] <= trace_bound+ROUND_OFF for c in ("ideal", "bias"))
    evidence["passed"] = all(evidence["checks"].values())
    sources = ("validate_gyro_bias_response.py", "test_gyro_bias_response.py", "gyro_sensor_model.py", "attitude_mekf.py",
        "attitude_mekf_adapter.py", "scenario_huskysat2_detumble.py", "config/attitude_mekf_test_only.json")
    report = dict(scope=SCOPE, passed=all(evidence["checks"].values()),
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        duration_s=DURATION_S, reference_preservation=preserved, configuration=ideal.attrs["simulation_config"],
        options=normalized(ideal.attrs["shadow_options"]), gyro_profiles={p:profile_config(p).to_dict() for p in ("IDEAL_REGRESSION","TEST_BIAS_ONLY")},
        estimator_policy=policy, covariance_attitude_trace_bound_rad2=float(trace_bound),
        evidence=evidence, host_sha256={name:hashlib.sha256(csv_bytes(f)).hexdigest() for name,f in (("ideal",ideal),("bias",biased))})
    return report, ideal, biased, iw, bw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase8a2a_gyro_bias.json")
    args = parser.parse_args()
    report, ideal, biased, _, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, default=json_value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for name, frame in (("ideal", ideal), ("bias", biased)):
        args.report.with_name(args.report.stem+"_"+name+".csv").write_bytes(csv_bytes(frame))
    print(json.dumps(dict(passed=report["passed"], checks=report["evidence"]["checks"],
        reference_preservation=report["reference_preservation"],
        ideal=report["evidence"]["ideal"]["summary"], bias=report["evidence"]["bias"]["summary"],
        short_interval=report["evidence"]["independent_short_interval"]), default=json_value, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
