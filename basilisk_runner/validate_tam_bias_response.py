"""8B-2A one unchanged synthetic TAM offset, shadow only, 2026-10-02.

Measurement composition precedes estimator interpretation. An uninitialized
filter is BLOCKED, never a passing zero-error response. No acquisition gate,
initial state, Q/R/P0, sensor schedule or controller is changed by this harness.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf_adapter import ShadowOptions
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from tam_sensor_model import profile_config
from validate_gyro_bias_response import LiveUpdateWitness, json_value, passive_rotation, angle_between
from validate_gyro_sensor_model import normalized, old_adapter_run
from validate_mekf_closed_loop import csv_bytes, mrp, vector
from validate_tam_sensor_model import events, invalid_cycle_metadata

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SCOPE = "SYNTHETIC TAM-BIAS ESTIMATOR RESPONSE / POST-ACQUISITION SHADOW DEVELOPMENT TEST / COLD-START ACQUISITION LIMIT IDENTIFIED / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"
DURATION_S = 3.0  # Unchanged 8B-1 horizon, not flight/convergence acceptance.
BIAS_S_T = np.array([1e-6, -2e-6, 3e-6])  # Existing 8B-1 TEST-ONLY fixture, not installed bias.
ENABLE_NS = 1_400_000_000  # User-selected existing acquisition; not flight timing.
ROUND_OFF = 256*np.finfo(float).eps  # Numerical checks only, never an innovation-acceptance threshold.


def run_case(profile="IDEAL_REGRESSION", *, rejected=False, enable_ns=0):
    witness = LiveUpdateWitness()
    with witness.record(), contextlib.redirect_stdout(io.StringIO()), invalid_cycle_metadata(rejected):
        frame = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), control_source="SIMPLE_NAV_REFERENCE",
            shadow=ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000,
                tam_model=profile_config(profile), tam_bias_test_only=profile == "TEST_BIAS_ONLY",tam_bias_enable_ns=enable_ns))
    return frame, witness


def composition(ideal, biased, enable_ns=0):
    """Independently compare the actual delivered vectors, not just model logs."""
    clean, changed = events(ideal), events(biased)
    records = biased.attrs["tam_model_samples"]
    cfg = profile_config("TEST_BIAS_ONLY")
    metadata = lambda t,e: (t,e.event_id,e.sensor,e.epoch_ns,e.reference_epoch_ns,e.frame,e.valid,e.invalid_reason)
    values, reconstruction, samples = [], [], []
    for (ti,a),(tb,b),record in zip(clean,changed,records):
        c_sb = np.asarray(a.c_sb)
        offset = BIAS_S_T if b.epoch_ns >= enable_ns else np.zeros(3)
        expected = np.asarray(a.measured)+offset
        # Independent scalar projections of S into B; do not use MEKF/model helpers.
        expected_b = [sum(c_sb[j,k]*expected[j] for j in range(3)) for k in range(3)]
        measured = record["sample"]
        values.append(np.array_equal(b.measured, expected) and np.array_equal(measured["measurement_S_T"], expected))
        reconstruction.append(np.array_equal(measured["reconstructed_B_T"],expected_b))
        samples.append(dict(event_id=b.event_id,acquisition_ns=b.epoch_ns,processing_ns=tb,
            expected_S_T=expected.tolist(),delivered_S_T=list(b.measured),expected_B_T=expected_b,
            applied_bias_S_T=(np.asarray(b.measured)-a.measured).tolist(),valid=b.valid))
    checks = dict(
        unchanged_TEST_ONLY_bias_profile=cfg.runtime_usable_for_flight is False
            and np.array_equal(cfg.bias_S.value,BIAS_S_T) and cfg.bias_S.units == "T" and cfg.bias_S.frame == "S"
            and np.array_equal(cfg.mounting_C_SB.value,np.eye(3)) and np.array_equal(cfg.scale.value,np.ones(3))
            and not np.any(cfg.cross_axis.value) and not np.any(cfg.noise_sigma_S.value)
            and cfg.seed.value is None and cfg.range_S.value is None and cfg.latency_ns.value == 0,
        exact_once_bias_in_delivered_sensor_and_body_vectors=len(clean) == len(changed) == len(records) == 3
            and all(values) and all(reconstruction),
        unchanged_event_frames_epochs_and_input_validity=[metadata(*e) for e in clean] == [metadata(*e) for e in changed]
            and all(np.array_equal(a.reference_n,b.reference_n) and np.array_equal(a.c_sb,b.c_sb)
                for (_,a),(_,b) in zip(clean,changed)),
        field_acquisition_publication_and_provenance=all(r["state_epoch_ns"] == r["sample"]["truth_epoch_ns"]
            == r["sample"]["acquisition_ns"] == r["sample"]["publication_ns"] == e.epoch_ns
            and r["bridge_delivery_ns"] == t and r["sample"]["configuration_fingerprint"]
            == profile_config("TEST_BIAS_ONLY" if e.epoch_ns >= enable_ns else "IDEAL_REGRESSION").fingerprint()
            and r["sample"]["units"] == "T" and not np.any(r["sample"]["noise_S_T"])
            and r["sample"]["valid"] and not r["sample"]["rejection_reasons"] for r,(t,e) in zip(records,changed)),
        gyro_and_Sun_paths_exact=all(np.array_equal(a.gyro_point_B,b.gyro_point_B)
            and np.array_equal(b.gyro_point_B,rate) and a.gyro_sample is None and b.gyro_sample is None
            and normalized(a.intervals) == normalized(b.intervals)
            and normalized([s for s in a.deliveries if s.sensor == "sun"]) == normalized([s for s in b.deliveries if s.sensor == "sun"])
            for a,b,rate in zip(ideal.attrs["shadow_input_trace"],biased.attrs["shadow_input_trace"],vector(biased,"omega_B","rad_s"))))
    return dict(passed=bool(all(checks.values())),checks={k:bool(v) for k,v in checks.items()},samples=samples,
        configuration=cfg.to_dict(),configuration_fingerprint=cfg.fingerprint())


def acquisition_geometry(frame, policy):
    """Rotation preserves pairwise angles; no TRIAD/MEKF equations duplicated."""
    first = next(b for b in frame.attrs["shadow_input_trace"] if len(b.deliveries) == 2)
    mag = next(e for e in first.deliveries if e.sensor == "magnetic")
    sun = next(e for e in first.deliveries if e.sensor == "sun")
    mb = np.asarray(mag.c_sb).T @ mag.measured
    sb = np.asarray(sun.measured)
    mn,sn = np.asarray(mag.reference_n),np.asarray(sun.reference_n)
    cosine_body = float(mb @ sb / (np.linalg.norm(mb)*np.linalg.norm(sb)))
    cosine_reference = float(mn @ sn / (np.linalg.norm(mn)*np.linalg.norm(sn)))
    gap = abs(cosine_body-cosine_reference)
    tolerance = policy["acquisition_pair_tolerance"]["value"]
    return dict(epoch_ns=first.sim_epoch_ns,body_pair_cosine=cosine_body,reference_pair_cosine=cosine_reference,
        absolute_cosine_disagreement=gap,existing_TEST_ONLY_tolerance=tolerance,
        consistent=gap <= tolerance,source="independent vector dot products at original acquisition epoch")


def summary(frame,witness):
    """Undefined estimator results remain None, never fabricated zero errors."""
    truth={int(t):np.asarray(rbk.MRP2C(s)) for t,s in zip(frame.time_ns,mrp(frame))}
    error=lambda core: float(np.linalg.norm(passive_rotation(np.asarray(rbk.EP2C(core["q_BN"])) @ truth[core["epoch_ns"]].T)))
    published=[r["core"] for r in frame.attrs["shadow_estimator_trace"] if r["core"] is not None]
    updates=[e for e in witness.events if e["initialized_before"] and e["result"] == "updated"]
    witnessed=published+[e[stage] for e in updates for stage in ("before","after")]
    status=frame.attrs["shadow_status"][-1]
    p=np.array([c["P"] for c in witnessed])
    return dict(final_attitude_error_rad=error(published[-1]) if published else None,
        max_published_attitude_error_rad=max(map(error,published)) if published else None,
        max_witnessed_attitude_error_rad=max(map(error,witnessed)) if witnessed else None,
        final_estimated_gyro_bias_B_rad_s=published[-1]["bias_B_rad_s"] if published else None,
        maximum_tangent_innovation={s:max((float(np.linalg.norm(e["after"]["last_update"]["tangent_innovation"]))
            for e in updates if e["sensor"] == s),default=None) for s in ("magnetic","sun")},
        covariance_min_eigenvalue=float(np.linalg.eigvalsh(p).min()) if witnessed else None,
        covariance_max_eigenvalue=float(np.linalg.eigvalsh(p).max()) if witnessed else None,
        covariance_max_asymmetry=float(np.max(np.abs(p-p.transpose(0,2,1)))) if witnessed else None,
        covariance_min_diagonal=float(np.diagonal(p,axis1=1,axis2=2).min()) if witnessed else None,
        maximum_attitude_covariance_trace=float(np.trace(p[:,:3,:3],axis1=1,axis2=2).max()) if witnessed else None,
        maximum_bias_covariance_diagonal=np.diagonal(p[:,3:,3:],axis1=1,axis2=2).max(axis=0) if witnessed else None,
        maximum_estimated_gyro_bias_norm_rad_s=max(float(np.linalg.norm(c["bias_B_rad_s"])) for c in witnessed) if witnessed else None,
        all_states_and_covariances_finite=bool(witnessed) and all(np.isfinite(c["q_BN"]).all()
            and np.isfinite(c["bias_B_rad_s"]).all() and np.isfinite(c["P"]).all() for c in witnessed),
        estimator_valid=status["valid"],rejected_events=status["rejected"],update_counts=status["updates"],
        replay_count=status["replays"],actual_update_callbacks=len(updates),
        gyro_bias_history=[dict(epoch_ns=c["epoch_ns"],bias_B_rad_s=c["bias_B_rad_s"]) for c in published])


def response(frame,witness,ideal,policy):
    """Read-only geometry of actual updates, including delayed replay callbacks.

    No Kalman gain, propagation or covariance update is duplicated. Tangent
    coordinates are decoded solely to compare the recorded innovation with a
    direct vector projection and the observed passive attitude rotation.
    """
    truth={int(t):np.asarray(rbk.MRP2C(s)) for t,s in zip(frame.time_ns,mrp(frame))}
    ideal_mag={e.epoch_ns:e for _,e in events(ideal)}
    sun_n=np.array(policy["sun_reference_n"]["value"]); sun_n/=np.linalg.norm(sun_n)
    unit=lambda x: np.asarray(x)/np.linalg.norm(x)
    rows=[]
    for event in witness.events:
        if not event["initialized_before"] or event["result"] != "updated":
            continue
        sample,pre,post=event["sample"],event["before"],event["after"]
        cb,ca=np.asarray(rbk.EP2C(pre["q_BN"])),np.asarray(rbk.EP2C(post["q_BN"]))
        raw=np.array(sample["measured"])
        if sample["frame"] == "S":
            raw=np.asarray(sample["c_sb"]).T @ raw
        measured,ref=unit(raw),unit(sample["reference_n"])
        predicted=cb @ ref
        # Only decode the documented two-component tangent coordinate basis.
        seed=np.eye(3)[int(np.argmin(np.abs(predicted)))]
        e1=unit(np.cross(predicted,seed)); e2=np.cross(predicted,e1)
        tangent=np.array(post["last_update"]["tangent_innovation"])
        innovation_B=e1*tangent[0]+e2*tangent[1]
        projected=(measured-predicted)-predicted*np.dot(predicted,measured-predicted)
        correction=passive_rotation(ca @ cb.T)
        descent=np.cross(measured,predicted)
        ct=truth[event["acquisition_ns"]]
        row=dict(callback_index=event["callback_index"],processing_ns=event["processing_ns"],
            acquisition_ns=event["acquisition_ns"],event_id=event["event_id"],sensor=event["sensor"],
            measured_B=raw.tolist(),measured_unit_B=measured.tolist(),reference_unit_N=ref.tolist(),predicted_unit_B=predicted.tolist(),
            tangent_innovation=tangent.tolist(),innovation_B=innovation_B.tolist(),independent_projected_residual_B=projected.tolist(),
            tangent_vector_discrepancy=float(np.max(np.abs(innovation_B-projected))),
            pre_residual_rad=angle_between(measured,predicted),post_residual_rad=angle_between(measured,ca @ ref),
            recorded_residual_rad=post["last_update"]["residual_angle_rad"],
            correction_B_rad=correction.tolist(),geometric_descent_B=descent.tolist(),descent_dot=float(correction @ descent),
            pre_truth_error_rad=float(np.linalg.norm(passive_rotation(cb @ ct.T))),
            post_truth_error_rad=float(np.linalg.norm(passive_rotation(ca @ ct.T))),
            pre_Sun_residual_rad=angle_between(ct @ sun_n,cb @ sun_n),
            post_Sun_residual_rad=angle_between(ct @ sun_n,ca @ sun_n),
            pre_estimated_bias_B_rad_s=pre["bias_B_rad_s"].tolist(),post_estimated_bias_B_rad_s=post["bias_B_rad_s"].tolist(),
            delta_bias_B_rad_s=(post["bias_B_rad_s"]-pre["bias_B_rad_s"]).tolist())
        if event["sensor"] == "magnetic":
            baseline=ideal_mag[event["acquisition_ns"]]
            ideal_body=np.asarray(baseline.c_sb).T @ baseline.measured
            offset_B=np.asarray(sample["c_sb"]).T @ BIAS_S_T
            expected=unit(ideal_body+offset_B)
            ideal_unit=unit(ideal_body)
            known_shift=expected-ideal_unit
            known_shift-=predicted*np.dot(predicted,known_shift)
            ideal_residual=ideal_unit-predicted
            ideal_residual-=predicted*np.dot(predicted,ideal_residual)
            introduced_axis=np.cross(expected,ideal_unit)
            row.update(ideal_B_T=ideal_body.tolist(),biased_B_T=raw.tolist(),bias_B_T=offset_B.tolist(),
                introduced_direction_angle_rad=angle_between(ideal_unit,expected),
                introduced_passive_axis_B=introduced_axis.tolist(),known_projected_bias_shift_B=known_shift.tolist(),
                innovation_minus_ideal_residual_B=(innovation_B-ideal_residual).tolist(),
                known_shift_discrepancy=float(np.max(np.abs(innovation_B-ideal_residual-known_shift))),
                innovation_dot_known_shift=float(innovation_B @ known_shift),
                correction_dot_introduced_axis=float(correction @ introduced_axis))
        rows.append(row)
    # Preserve every callback; LAST occurrence is the final retained replay history.
    unique=sorted({u["event_id"]:u for u in rows}.values(),key=lambda u:u["acquisition_ns"])
    magnetic=[u for u in unique if u["sensor"] == "magnetic"]
    following_sun=[u for u in unique if u["sensor"] == "sun" and u["acquisition_ns"] > ENABLE_NS]
    stats=summary(frame,witness)
    history=[r["core"] for r in frame.attrs["shadow_estimator_trace"] if r["core"] is not None]
    update_ticks={u["processing_ns"] for u in rows}
    last_at_tick={u["processing_ns"]:u for u in rows}
    p0=np.array(policy["p0_diagonal"]["value"])
    trace_bound=2*p0[:3].sum()+2*DURATION_S**2*p0[3:].sum()  # Existing zero-Q kinematic bound.
    checks=dict(
        actual_magnetic_and_Sun_innovations_match_vector_geometry=bool(rows) and all(
            u["tangent_vector_discrepancy"] <= ROUND_OFF
            and abs(u["recorded_residual_rad"]-u["pre_residual_rad"]) <= ROUND_OFF
            and u["descent_dot"] >= -ROUND_OFF**2 for u in rows),
        actual_updates_reduce_their_measured_vector_residual=bool(rows) and all(
            u["post_residual_rad"] <= u["pre_residual_rad"]+ROUND_OFF for u in rows),
        known_bias_innovation_sign_and_first_correction=len(magnetic) == 2
            and all(u["known_shift_discrepancy"] <= ROUND_OFF for u in magnetic)
            and magnetic[0]["innovation_dot_known_shift"] > 0 and magnetic[0]["correction_dot_introduced_axis"] > 0,
        ideal_Sun_observes_and_corrects_magnetic_disagreement=len(following_sun) == 2
            and magnetic[0]["post_Sun_residual_rad"] > magnetic[0]["pre_Sun_residual_rad"]
            and all(u["post_Sun_residual_rad"] < u["pre_Sun_residual_rad"] for u in following_sun),
        covariance_finite_symmetric_positive_definite=stats["all_states_and_covariances_finite"]
            and stats["covariance_max_asymmetry"] <= ROUND_OFF
            and stats["covariance_min_eigenvalue"] > ROUND_OFF and stats["covariance_min_diagonal"] >= 0,
        covariance_within_unchanged_zero_Q_bound=not np.any(policy["qc_diagonal"]["value"])
            and np.all(stats["maximum_bias_covariance_diagonal"] <= p0[3:]+ROUND_OFF)
            and stats["maximum_attitude_covariance_trace"] <= trace_bound+ROUND_OFF,
        estimated_gyro_bias_changes_only_in_observed_vector_updates=all(
            np.array_equal(a["bias_B_rad_s"],b["bias_B_rad_s"]) for a,b in zip(history,history[1:]) if b["epoch_ns"] not in update_ticks)
            and all(np.array_equal(c["bias_B_rad_s"],last_at_tick[c["epoch_ns"]]["post_estimated_bias_B_rad_s"])
                for c in history if c["epoch_ns"] in last_at_tick),
        initialized_response_valid_without_rejected_events=all(s["valid"] for s in frame.attrs["shadow_status"][4:])
            and stats["rejected_events"] == {} and stats["update_counts"] == {"magnetic":3,"sun":4} and stats["replay_count"] == 2)
    return dict(passed=bool(all(checks.values())),checks={k:bool(v) for k,v in checks.items()},summary=stats,
        all_update_callbacks=rows,unique_updates=unique,first_biased_update=magnetic[0] if magnetic else None,
        covariance_attitude_trace_bound_rad2=float(trace_bound))


def validate():
    ideal,iw = run_case()
    biased,bw = run_case("TEST_BIAS_ONLY")
    seq = composition(ideal,biased)
    post,pw = run_case("TEST_BIAS_ONLY",enable_ns=ENABLE_NS)
    post_seq = composition(ideal,post,ENABLE_NS)
    policy = json.loads((HERE/"config/attitude_mekf_test_only.json").read_text(encoding="utf-8"))
    report = dict(scope=SCOPE,passed=False,engineering_gate="INVESTIGATE",checks=dict(seq["checks"]),composition=seq,
        post_acquisition_composition=post_seq,cold_start_bias_acquisition_supported=False,
        base_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        duration_s=DURATION_S,estimator_policy=policy,cycle_config=diagnostic_cycle_config().to_dict(),
        simulation_config=ideal.attrs["simulation_config"],
        options={n:f.attrs["shadow_options"] for n,f in (("ideal",ideal),("cold_start_bias",biased),("post_acquisition_bias",post))})
    report["checks"]["post_acquisition_composition_hard_gate"] = post_seq["passed"]
    prior_path=HERE/"output_data/phase8b2a_tam_bias.json"
    if prior_path.exists():
        prior=json.loads(prior_path.read_text(encoding="utf-8"))
        discovery=prior.get("cold_start_discovery",prior if prior.get("engineering_gate") == "BLOCKED" else None)
        if discovery is not None:
            report["cold_start_discovery"]=discovery  # Retain original nonzero/BLOCKED artifact, without recursive nesting.
    # Hard gate: never analyze a response to an incorrectly composed measurement.
    if not seq["passed"] or not post_seq["passed"]:
        report["why"] = "Measurement composition failed; estimator interpretation withheld"
        return report,ideal,biased,iw,bw,post,pw
    with contextlib.redirect_stdout(io.StringIO()):
        committed = old_adapter_run(dict(ideal_sun=True,magnetic_delay_ns=200_000_000,tam_model=profile_config()))
    rejected,rw = run_case("TEST_BIAS_ONLY",rejected=True,enable_ns=ENABLE_NS)
    rejected_sample = rejected.attrs["tam_model_samples"][1]["sample"]
    rejected_event = events(rejected)[1][1]
    guards=[]
    for enable in (0,ENABLE_NS):
        try:
            run(stop_time_s=3.,write_outputs=False,make_plots=False,cycle=diagnostic_cycle_config(),
                control_source="MEKF_DEVELOPMENT",shadow=ShadowOptions(ideal_sun=True,
                    tam_model=profile_config("TEST_BIAS_ONLY"),tam_bias_test_only=True,tam_bias_enable_ns=enable))
            guards.append(False)
        except ValueError as error:
            guards.append("MEKF_DEVELOPMENT requires" in str(error))
    unchanged = ("tam_sensor_model.py","attitude_mekf.py","attitude_mekf_prototype.py","gyro_sensor_model.py",
        "magnetic_control_cycle.py","magnetic_environment.py","basilisk_adcs_adapter.py","hs2_sim_config.py",
        "scenario_huskysat2_detumble.py","config/attitude_mekf_test_only.json","validate_gyro_bias_response.py")
    checks=report["checks"]
    checks.update(ideal_reference_matches_committed_8B1=csv_bytes(ideal) == csv_bytes(committed)
        and normalized(ideal.attrs["shadow_input_trace"]) == normalized(committed.attrs["shadow_input_trace"])
        and normalized(ideal.attrs["shadow_estimator_trace"]) == normalized(committed.attrs["shadow_estimator_trace"]),
        native_control_spacecraft_and_actuators_byte_identical=csv_bytes(ideal) == csv_bytes(biased) == csv_bytes(post) == csv_bytes(rejected),
        no_development_command_owner=all("mekf_command_owner" not in f.attrs for f in (ideal,biased,post,rejected)),
        biased_TAM_actuator_selection_rejected=all(guards),
        rejected_finite_biased_vector_preserved=not rejected_sample["valid"] and bool(rejected_sample["rejection_reasons"])
            and np.array_equal(rejected_sample["measurement_S_T"],biased.attrs["tam_model_samples"][1]["sample"]["measurement_S_T"])
            and not rejected_event.valid and rejected_event.invalid_reason == "magnetic_cycle_invalid"
            and np.array_equal(rejected_event.measured,rejected_sample["measurement_S_T"])
            and rejected.attrs["shadow_status"][-1]["rejected"].get("magnetic_cycle_invalid") == 1,
        physical_models_controller_cycle_and_Q_R_P0_unchanged=subprocess.run(["git","diff","--quiet","HEAD","--",
            *("basilisk_runner/"+n for n in unchanged)],cwd=ROOT).returncode == 0)
    geometry = {n:acquisition_geometry(f,policy) for n,f in (("ideal",ideal),("bias",biased))}
    bias_status=biased.attrs["shadow_status"][-1]
    blocked = (not geometry["bias"]["consistent"] and not bias_status["initialized"]
        and bias_status["rejected"].get("inconsistent_acquisition_pair") == 1)
    checks["cold_start_rejection_explained_by_independent_pair_geometry"] = blocked and geometry["ideal"]["consistent"]
    initialized=[s["sim_epoch_ns"] for s in post.attrs["shadow_status"] if s["acquisition_event"]]
    biased_epochs=[r["sample"]["acquisition_ns"] for r in post.attrs["tam_model_samples"]
        if r["sample"]["configuration_fingerprint"] == profile_config("TEST_BIAS_ONLY").fingerprint()]
    checks["normal_ideal_acquisition_before_explicit_bias_boundary"] = initialized == [400_000_000]
    checks["initial_ideal_then_only_two_subsequent_biased_acquisitions"] = biased_epochs == [ENABLE_NS,2_400_000_000]
    checks["all_estimator_states_exact_before_first_biased_delivery"] = all(normalized(a) == normalized(b)
        for a,b in zip(ideal.attrs["shadow_estimator_trace"][:16],post.attrs["shadow_estimator_trace"][:16]))
    healthy_start=all(s["valid"] for s in post.attrs["shadow_status"][4:])
    observed=response(post,pw,ideal,policy) if healthy_start else None
    if observed is not None:
        checks.update(observed["checks"])
    else:
        checks["post_acquisition_estimator_available_for_analysis"]=False
    report.update(passed=bool(all(checks.values())),engineering_gate="PASS" if all(checks.values()) else "INVESTIGATE" if healthy_start else "BLOCKED",
        why="Post-acquisition bias response must agree with independent vector geometry; cold-start acquisition remains unsupported",
        acquisition_geometry=geometry,
        post_acquisition_response=observed,
        transition=dict(ideal_acquisition_ns=400_000_000,initialization_epochs_ns=initialized,bias_enable_ns=ENABLE_NS,
            biased_acquisitions_ns=biased_epochs,biased_processing_epochs_ns=[t for t,e in events(post) if e.epoch_ns >= ENABLE_NS]),
        summaries={n:summary(f,w) for n,f,w in (("ideal",ideal,iw),("bias",biased,bw),("post_acquisition_bias",post,pw))},
        final_status={n:f.attrs["shadow_status"][-1] for n,f in (("ideal",ideal),("bias",biased),("post_acquisition_bias",post))},
        model_samples={n:f.attrs["tam_model_samples"] for n,f in (("ideal",ideal),("bias",biased),("post_acquisition_bias",post),("rejected_bias",rejected))},
        estimator_trace={n:normalized(f.attrs["shadow_estimator_trace"]) for n,f in (("ideal",ideal),("bias",biased),("post_acquisition_bias",post))},
        input_trace={n:normalized(f.attrs["shadow_input_trace"]) for n,f in (("ideal",ideal),("bias",biased),("post_acquisition_bias",post))},
        raw_observe_callbacks={n:normalized(w.events) for n,w in (("ideal",iw),("bias",bw),("post_acquisition_bias",pw),("rejected_bias",rw))},
        host_sha256={n:hashlib.sha256(csv_bytes(f)).hexdigest() for n,f in (("committed",committed),("ideal",ideal),("bias",biased),("post_acquisition_bias",post),("rejected_bias",rejected))},
        source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in unchanged+(
            "attitude_mekf_adapter.py","validate_tam_bias_response.py","test_tam_bias_response.py","validate_tam_sensor_model.py","validate_gyro_sensor_model.py")})
    return report,ideal,biased,iw,bw,post,pw


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report",type=Path,default=HERE/"output_data/phase8b2a_tam_bias.json")
    args=parser.parse_args()
    report,ideal,biased,_,_,post,_=validate()
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,default=json_value,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    for n,f in (("ideal",ideal),("bias",biased),("post_acquisition_bias",post)):
        args.report.with_name(args.report.stem+"_"+n+".csv").write_bytes(csv_bytes(f))
    print(json.dumps({k:report[k] for k in ("passed","engineering_gate","why","checks","cold_start_bias_acquisition_supported","acquisition_geometry","transition") if k in report},default=json_value,indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
