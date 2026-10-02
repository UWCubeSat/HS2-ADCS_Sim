"""8B-1 short ideal TAM equivalence and preservation; no perturbation campaign."""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
from unittest.mock import patch

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import magnetometer
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf_adapter import IdealLiveBridge, ShadowOptions
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import DEFAULT_CONFIG, run
from tam_sensor_model import SCOPE, TAMConfig, TAMModel, profile_config
from validate_gyro_sensor_model import normalized, old_adapter_run
from validate_gyro_bias_response import json_value
from validate_mekf_closed_loop import csv_bytes, mrp, vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ROUND_OFF = 128*np.finfo(float).eps
ESTIMATOR_TOLERANCE = 1e-12  # Existing development equivalence tolerance; no accuracy claim.


@contextlib.contextmanager
def invalid_cycle_metadata(enabled):
    """One shadow-only rejection witness; restore bookkeeping before control.

    At bridge priority 590 the driver has completed its acquisition. Temporarily
    invalidate its metadata for the bridge read only, never alter dipole/field
    messages, driver.valid or subsequent control. Both A/B cases get this fault.
    """
    original = IdealLiveBridge.magnetic_acquisition
    def acquire(bridge, tick):
        if tick != 1_400_000_000:
            return original(bridge, tick)
        row = bridge.driver.history[-1]
        saved = row["cycle_sample_valid"]
        try:
            row["cycle_sample_valid"] = False
            return original(bridge, tick)
        finally:
            row["cycle_sample_valid"] = saved
    with patch.object(IdealLiveBridge, "magnetic_acquisition", acquire) if enabled else contextlib.nullcontext():
        yield


def run_case(modeled=False, rejected=False):
    with contextlib.redirect_stdout(io.StringIO()), invalid_cycle_metadata(rejected):
        return run(stop_time_s=3., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            control_source="SIMPLE_NAV_REFERENCE", shadow=ShadowOptions(ideal_sun=True,
                magnetic_delay_ns=200_000_000, tam_model=TAMConfig() if modeled else None))


def events(frame):
    return [(b.sim_epoch_ns, e) for b in frame.attrs["shadow_input_trace"] for e in b.deliveries if e.sensor == "magnetic"]


def compare(a, b):
    ea, eb = events(a), events(b)
    metadata = lambda delivery,e: (delivery,e.event_id,e.sensor,e.epoch_ns,e.reference_epoch_ns,e.frame,e.valid,e.invalid_reason)
    event_metadata_equal = [metadata(*e) for e in ea] == [metadata(*e) for e in eb]
    differences = [float(np.max(np.abs(np.array(x.measured)-y.measured))) for (_,x),(_,y) in zip(ea,eb)]
    field_scale = max(float(np.linalg.norm(e.measured)) for _,e in ea)
    maxima = dict(q_BN=0., bias_B_rad_s=0., P=0.)
    discrete_equal = len(a.attrs["shadow_estimator_trace"]) == len(b.attrs["shadow_estimator_trace"]) == len(a)
    for old,new in zip(a.attrs["shadow_estimator_trace"], b.attrs["shadow_estimator_trace"]):
        x,y = old["core"],new["core"]
        if (x is None) != (y is None):
            discrete_equal = False
        if x is not None and y is not None:
            for key in maxima:
                maxima[key] = max(maxima[key],float(np.max(np.abs(np.asarray(x[key])-y[key]))))
            discrete_equal &= x["epoch_ns"] == y["epoch_ns"] and old["updates"] == new["updates"]
    statuses_equal = len(a.attrs["shadow_status"]) == len(b.attrs["shadow_status"]) == len(a)
    for x,y in zip(a.attrs["shadow_status"],b.attrs["shadow_status"]):
        for key in ("valid","state","state_epoch_ns","rejected","updates","replays","sensor_epochs","accepted_sensor_epochs"):
            statuses_equal &= x[key] == y[key]
        # source strings intentionally identify the opt-in value producer.
        statuses_equal &= [{k:v for k,v in item.items() if k != "source"} for item in x["results"]] == [
            {k:v for k,v in item.items() if k != "source"} for item in y["results"]]
    checks = dict(sample_sequence_epochs_frames_validity_exact=event_metadata_equal,
        magnetic_vectors_equal_to_transform_roundoff=max(differences) <= ROUND_OFF*field_scale,
        reference_vectors_and_mounting_exact=all(np.array_equal(x.reference_n,y.reference_n) and np.array_equal(x.c_sb,y.c_sb)
            for (_,x),(_,y) in zip(ea,eb)),
        estimator_q_bias_covariance_equivalent=all(x <= ESTIMATOR_TOLERANCE for x in maxima.values()),
        estimator_event_order_epochs_status_counts_exact=bool(discrete_equal and statuses_equal),
        host_CSV_byte_identical=csv_bytes(a) == csv_bytes(b))
    return dict(passed=all(checks.values()), checks=checks, magnetic_max_abs_difference_T=max(differences),
        magnetic_tolerance_T=ROUND_OFF*field_scale, estimator_max_abs_differences=maxima,
        estimator_numerical_tolerance=ESTIMATOR_TOLERANCE,
        events=[dict(processing_ns=t,event_id=e.event_id,acquisition_ns=e.epoch_ns,valid=e.valid,reason=e.invalid_reason)
            for t,e in eb], final_status=b.attrs["shadow_status"][-1])


def native_history_equivalence(frame):
    """All 31 WMM/state epochs against an independently executed native sensor.

    Outside the plant, no actuator/MEKF connection: this checks transform/value
    equivalence between Python and Basilisk C++ on the actual recorded history.
    No quiet eligibility is claimed for these value-only samples.
    """
    state, field = messaging.SCStatesMsg(), messaging.MagneticFieldMsg()
    native = magnetometer.Magnetometer()
    native.stateInMsg.subscribeTo(state)
    native.magInMsg.subscribeTo(field)
    config = DEFAULT_CONFIG.sensors
    native.dcm_SB = [list(row) for row in config.magnetometer_dcm_SB.value]
    native.scaleFactor = config.magnetometer_scale.value
    native.senBias = list(config.magnetometer_bias.value)
    native.senNoiseStd = list(config.magnetometer_noise_std.value)
    native.minOutput, native.maxOutput = config.magnetometer_min_output.value,config.magnetometer_max_output.value
    native.Reset(0)
    model = TAMModel(TAMConfig())
    differences, scale = [], []
    for tick,sigma,bn in zip(frame.time_ns.to_numpy(dtype=np.int64), mrp(frame), vector(frame,"B_N","T")):
        sp,mp = messaging.SCStatesMsgPayload(),messaging.MagneticFieldMsgPayload()
        sp.sigma_BN,mp.magField_N = sigma.tolist(),bn.tolist()
        state.write(sp,int(tick)); field.write(mp,int(tick))
        native.UpdateState(int(tick))
        body = np.asarray(rbk.MRP2C(sigma)) @ bn
        result = model.acquire(body,truth_epoch_ns=int(tick),acquisition_ns=int(tick),context=None,
            field_source="recorded live WMM/state history, value comparison only")
        differences.append(float(np.max(np.abs(np.array(result.measurement_S_T)-native.tamDataOutMsg.read().tam_S))))
        scale.append(float(np.linalg.norm(body)))
    tolerance = ROUND_OFF*max(scale)
    return dict(passed=max(differences) <= tolerance, samples=len(differences),
        max_abs_difference_T=max(differences), tolerance_T=tolerance)


def validate():
    # Read-only HEAD adapter loading establishes feature-disabled preservation.
    with contextlib.redirect_stdout(io.StringIO()):
        committed = old_adapter_run(dict(ideal_sun=True, magnetic_delay_ns=200_000_000))
    native, modeled = run_case(),run_case(True)
    rejected_native,rejected_model = run_case(rejected=True),run_case(True,True)
    nominal,rejection = compare(native,modeled),compare(rejected_native,rejected_model)
    history = native_history_equivalence(native)
    records = modeled.attrs["tam_model_samples"]
    rejected_records = rejected_model.attrs["tam_model_samples"]
    live_guards, authority_guards = [],[]
    for name in ("IDEAL_REGRESSION","TEST_BIAS_ONLY","TEST_SCALE_ONLY","TEST_NOISE_ONLY"):
        cfg = profile_config(name)
        if name != "IDEAL_REGRESSION":
            try:
                ShadowOptions(tam_model=cfg).validate(100_000_000)
                live_guards.append(False)
            except ValueError as error:
                live_guards.append("live TAM permits only" in str(error))
        try:
            run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
                shadow=ShadowOptions(ideal_sun=True,tam_model=cfg),control_source="MEKF_DEVELOPMENT")
            authority_guards.append(False)
        except ValueError as error:
            authority_guards.append("MEKF_DEVELOPMENT requires" in str(error))
    checks = dict(live_ideal_equivalence=nominal["passed"], rejected_sequence_equivalence=rejection["passed"],
        representative_WMM_history_equivalence=history["passed"],
        disabled_host_matches_HEAD_adapter=csv_bytes(committed) == csv_bytes(native),
        disabled_estimator_inputs_states_exact=normalized(committed.attrs["shadow_input_trace"]) == normalized(native.attrs["shadow_input_trace"])
            and normalized(committed.attrs["shadow_estimator_trace"]) == normalized(native.attrs["shadow_estimator_trace"]),
        all_acquisition_field_state_publication_epochs_equal=all(r["state_epoch_ns"] == r["sample"]["truth_epoch_ns"]
            == r["sample"]["acquisition_ns"] == r["sample"]["publication_ns"] for r in records),
        delayed_processing_keeps_acquisition_epoch=all(r["bridge_delivery_ns"] == t and r["sample"]["acquisition_ns"] == e.epoch_ns
            for r,(t,e) in zip(records,events(modeled))) and records[1]["bridge_delivery_ns"]-records[1]["sample"]["acquisition_ns"] == 200_000_000,
        ideal_model_no_rng_and_matching_provenance=TAMModel(TAMConfig()).rng is None
            and all(r["sample"]["configuration_fingerprint"] == TAMConfig().fingerprint() for r in records),
        ineligible_finite_measurement_preserved=not rejected_records[1]["sample"]["valid"]
            and np.array_equal(rejected_records[1]["sample"]["measurement_S_T"],records[1]["sample"]["measurement_S_T"])
            and np.any(rejected_records[1]["sample"]["measurement_S_T"])
            and rejection["final_status"]["rejected"].get("magnetic_cycle_invalid") == 1)
    # Keep ownership checks separate from metadata/value equivalence.
    checks.update(no_development_owner=all("mekf_command_owner" not in f.attrs for f in (native,modeled,rejected_model)),
        perturbed_TAM_live_selection_rejected=all(live_guards), all_TAM_closed_loop_selection_rejected=all(authority_guards),
        shadow_fault_does_not_change_actuation=csv_bytes(native) == csv_bytes(rejected_native) == csv_bytes(rejected_model))
    sources = ("tam_sensor_model.py","test_tam_sensor_model.py","validate_tam_sensor_model.py","test_tam_shadow_integration.py",
        "attitude_mekf_adapter.py","scenario_huskysat2_detumble.py","magnetic_control_cycle.py","attitude_mekf.py",
        "attitude_mekf_prototype.py","config/attitude_mekf_test_only.json")
    report = dict(scope=SCOPE,passed=bool(all(checks.values())),checks={k:bool(v) for k,v in checks.items()},
        base_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        profiles={name:profile_config(name).to_dict() for name in ("IDEAL_REGRESSION","TEST_BIAS_ONLY","TEST_SCALE_ONLY","TEST_NOISE_ONLY")},
        simulation_config=native.attrs["simulation_config"],cycle_config=diagnostic_cycle_config().to_dict(),
        nominal=nominal,rejection=rejection,native_WMM_history=history,
        model_samples=records,rejected_model_samples=rejected_records,
        native_estimator_trace=normalized(native.attrs["shadow_estimator_trace"]),
        modeled_estimator_trace=normalized(modeled.attrs["shadow_estimator_trace"]),
        host_sha256={name:hashlib.sha256(csv_bytes(f)).hexdigest() for name,f in (("committed",committed),("native",native),("modeled",modeled))})
    return report,native,modeled


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report",type=Path,default=HERE/"output_data/phase8b1_tam_model.json")
    args = parser.parse_args()
    report,native,modeled = validate()
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,default=json_value,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    for name,frame in (("native",native),("modeled",modeled)):
        args.report.with_name(args.report.stem+"_"+name+".csv").write_bytes(csv_bytes(frame))
    print(json.dumps({k:report[k] for k in ("passed","checks","nominal","rejection","native_WMM_history","host_sha256")},default=json_value,indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
