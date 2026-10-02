"""8B-2B cold-start pair geometry only, 2026-10-02; no tuning/performance study.

Replay the source-bound 8B-2A 0.4 s pair through the unchanged adapter/MEKF.
An independent 70-digit Decimal predicate never calls estimator math helpers.
Deterministic perturbations use the unchanged isolated TAM value model; no live
profile, actuator owner, scenario scheduling or flight configuration is changed.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict, replace
from decimal import Decimal, localcontext
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess

import numpy as np

from attitude_mekf import VectorSample
from attitude_mekf_adapter import DevelopmentChannel, InputBatch, MEKFNavigationAdapter, ShadowOptions
from hs2_sim_config import Parameter
from tam_sensor_model import TAMConfig, TAMModel, CleanlinessContext

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
SCOPE="COLD-START ACQUISITION-CONSISTENCY BOUNDARY / DEVELOPMENT CHARACTERIZATION / NO TUNING PERFORMED / NOT FLIGHT VALIDATED"
NUMERIC_MARGIN=16*np.finfo(float).eps  # Predicate-arithmetic resolution allowance, NOT an acquisition gate.


def json_value(x):
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,np.generic): return x.item()
    raise TypeError(type(x).__name__)


def batch(raw):
    data=deepcopy(raw)
    data["gyro_point_B"]=np.array(data["gyro_point_B"])
    data["deliveries"]=[VectorSample(**s) for s in data["deliveries"]]
    return InputBatch(**data)


def fixture():
    path=HERE/"output_data/phase8b2a_tam_bias.json"
    prior=json.loads(path.read_text(encoding="utf-8"))
    if not prior["passed"] or not all(hashlib.sha256((HERE/n).read_bytes()).hexdigest()==h
            for n,h in prior["source_sha256"].items()):
        raise ValueError("8B-2A acquisition evidence missing its passing source binding; do not silently use stale geometry")
    acquisition=next(b for b in prior["input_trace"]["ideal"] if b["sim_epoch_ns"]==400_000_000)
    sample=prior["model_samples"]["ideal"][0]
    policy=json.loads((HERE/"config/attitude_mekf_test_only.json").read_text(encoding="utf-8"))
    if sample["sample"]["acquisition_ns"]!=400_000_000 or not sample["sample"]["valid"]:
        raise ValueError("valid original 0.4 s TAM witness required")
    mag=next(s for s in acquisition["deliveries"] if s["sensor"]=="magnetic")
    if not np.array_equal(mag["c_sb"],TAMConfig().mounting_C_SB.value):
        raise ValueError("saved sensor mounting differs from the unchanged isolated model fixture")
    return dict(pair=batch(acquisition),idle=[batch(b) for b in prior["input_trace"]["ideal"] if b["sim_epoch_ns"]<400_000_000],
        body_T=np.array(sample["body_truth_T"]),context=CleanlinessContext(**sample["sample"]["cleanliness_context"]),
        policy=policy,source=dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_sha256=prior["source_sha256"],base_commit=prior["base_commit"]),original_gap=prior["acquisition_geometry"]["bias"]["absolute_cosine_disagreement"])


def independent(magnetic_B,sun_B,magnetic_N,sun_N,tolerance,minimum_sine):
    """Exact mathematical predicate, evaluated on the supplied floats at 70 digits.

    Normalized pairwise dot products only; no MEKF/VectorSample/unit/triad helper.
    Epoch, frame, quiet-validity and source prerequisites are checked separately.
    """
    with localcontext() as ctx:
        ctx.prec=70
        def unit(v):
            a=[Decimal.from_float(float(x)) for x in v]
            norm=sum(x*x for x in a).sqrt()
            return None if not norm else [x/norm for x in a]
        mb,sb,mn,sn=[unit(v) for v in (magnetic_B,sun_B,magnetic_N,sun_N)]
        if any(v is None for v in (mb,sb,mn,sn)):
            return dict(decision="REJECT",reason="finite nonzero vector required",metric=None)
        dot=lambda a,b: sum(x*y for x,y in zip(a,b))
        cross=lambda a,b: [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
        cb,cn=dot(mb,sb),dot(mn,sn)
        gap=abs(cb-cn)
        sines=[sum(x*x for x in cross(a,b)).sqrt() for a,b in ((mb,sb),(mn,sn))]
        reason="inconsistent_acquisition_pair" if gap>Decimal.from_float(tolerance) else (
            "acquisition_geometry" if min(sines)<=Decimal.from_float(minimum_sine) else "")
        return dict(decision="REJECT" if reason else "ACCEPT",reason=reason,metric=float(gap),metric_decimal=str(gap),
            body_cosine=float(cb),reference_cosine=float(cn),body_sine=float(sines[0]),reference_sine=float(sines[1]))


def actual(f,pair,*,reverse=False,idle=False):
    """Fresh uninitialized production adapter; no propagation after acquisition."""
    channel=DevelopmentChannel()
    adapter=MEKFNavigationAdapter(channel,ShadowOptions(ideal_sun=True))
    for old in f["idle"] if idle else ():
        channel.write(old,old.sim_epoch_ns)
        adapter.UpdateState(old.sim_epoch_ns)
    candidate=replace(pair,deliveries=list(reversed(pair.deliveries)) if reverse else list(pair.deliveries))
    original={e.sensor:(list(e.measured),e.valid) for e in candidate.deliveries}
    channel.write(candidate,candidate.sim_epoch_ns)
    adapter.UpdateState(candidate.sim_epoch_ns)
    status=adapter.history[-1]
    unchanged=all((list(e.measured),e.valid)==original[e.sensor] for e in adapter.input_trace[-1].deliveries)
    return dict(decision="ACCEPT" if status["initialized"] else "REJECT",rejected=status["rejected"],
        initialized=status["initialized"],input_validity_and_values_unchanged=unchanged,
        sensor_valid={e.sensor:e.valid for e in candidate.deliveries})


def geometry(f):
    mag,sun=(next(e for e in f["pair"].deliveries if e.sensor==s) for s in ("magnetic","sun"))
    b=f["body_T"]; g=float(np.linalg.norm(b)); ep=b/g
    s=np.array(sun.measured); s/=np.linalg.norm(s)
    normal=np.cross(ep,s); sine=float(np.linalg.norm(normal))
    if sine<=f["policy"]["minimum_acquisition_sine"]["value"]:
        raise ValueError("original geometry cannot support stable B/S basis")
    ec=normal/sine; ei=np.cross(ec,ep); ei/=np.linalg.norm(ei)
    return dict(field_B_T=b,field_magnitude_T=g,Sun_unit_B=s,field_N_T=mag.reference_n,Sun_reference_N=sun.reference_n,
        C_SB=mag.c_sb,parallel_B=ep,in_plane_B=ei,cross_plane_B=ec,
        included_angle_rad=float(math.atan2(sine,np.dot(ep,s))),cosine=float(np.dot(ep,s)),sine=sine)


def evaluate(f,g,bias_B_T,label,*,run_actual=True):
    mag,sun=(next(e for e in f["pair"].deliveries if e.sensor==s) for s in ("magnetic","sun"))
    c=np.array(mag.c_sb)
    offset=c @ np.asarray(bias_B_T)
    parameter=Parameter(tuple(float(x) for x in offset),"T","ASSUMED","Phase 8B-2B validate_acquisition_boundary.py",
        "2026-10-02","S","TEST-ONLY deterministic geometric characterization; not installed calibration")
    config=replace(TAMConfig(),profile="TEST_ONLY_ACQUISITION_BOUNDARY",bias_S=parameter)
    sample=TAMModel(config).acquire(f["body_T"],truth_epoch_ns=mag.epoch_ns,acquisition_ns=mag.epoch_ns,
        context=f["context"],field_source="source-bound 8B-2A ideal 0.4 s WMM/body field; deterministic isolated bias")
    pair=replace(f["pair"],deliveries=[replace(e,measured=sample.measurement_S_T,valid=sample.valid,
        source="8B-2B deterministic TAM fixture; unchanged acquisition epoch/quiet context") if e.sensor=="magnetic" else e
        for e in f["pair"].deliveries])
    body=c.T @ sample.measurement_S_T
    predicted=independent(body,sun.measured,mag.reference_n,sun.reference_n,
        f["policy"]["acquisition_pair_tolerance"]["value"],f["policy"]["minimum_acquisition_sine"]["value"])
    magnitude=float(np.linalg.norm(bias_B_T))
    unit=lambda a: np.array(a)/np.linalg.norm(a)
    binary_metric=None if not np.linalg.norm(body) else abs(float(unit(body) @ unit(sun.measured)-unit(mag.reference_n) @ unit(sun.reference_n)))
    result=dict(label=label,bias_B_T=list(bias_B_T),bias_S_T=offset.tolist(),bias_magnitude_T=magnitude,
        fraction_of_local_field=magnitude/g["field_magnitude_T"],measurement_S_T=sample.measurement_S_T,
        magnetic_direction_change_rad=None if not np.linalg.norm(body) else float(math.atan2(np.linalg.norm(np.cross(f["body_T"],body)),np.dot(f["body_T"],body))),
        sensor_valid=sample.valid,sensor_reasons=sample.rejection_reasons,configuration_fingerprint=config.fingerprint(),
        independent=predicted,binary64_metric=binary_metric,
        binary64_minus_independent=None if binary_metric is None else binary_metric-predicted["metric"],
        acquisition_ns=sample.acquisition_ns,publication_ns=sample.publication_ns,
        cycle_fingerprint=sample.cleanliness_context.cycle_fingerprint)
    if run_actual:
        observed=actual(f,pair)
        result["actual"]=observed
        result["agrees"]=predicted["decision"]==observed["decision"] and (not predicted["reason"] or predicted["reason"] in observed["rejected"])
    return result,pair


def boundary(f,g,direction_B,label,*,scale_T):
    """Local first-departure bracket, stopping above roundoff ambiguity.

    Only actual-tested points define the accepted/rejected endpoints. An
    arithmetic-band midpoint is retained as a geometry-only diagnostic, never
    mislabeled as a production acceptance decision.
    """
    tolerance=f["policy"]["acquisition_pair_tolerance"]["value"]
    def probe(x,run=True):
        row,pair=evaluate(f,g,x*scale_T*direction_B,label,run_actual=run)
        row["scale"]=x
        return row,pair
    low,lp=probe(0.)
    hi=1.
    high,hp=probe(hi)
    if high["actual"]["decision"]!="REJECT":
        raise ValueError("direction did not bracket a first local rejection")
    trace=[low,high]
    for _ in range(100):
        x=(low["scale"]+high["scale"])/2
        mid,mp=probe(x,False)
        if abs(mid["independent"]["metric"]-tolerance)<=NUMERIC_MARGIN:
            break
        observed=actual(f,mp)
        mid["actual"]=observed
        mid["agrees"]=mid["independent"]["decision"]==observed["decision"] and (
            not mid["independent"]["reason"] or mid["independent"]["reason"] in observed["rejected"])
        trace.append(mid)
        if observed["decision"]=="ACCEPT": low,lp=mid,mp
        else: high,hp=mid,mp
    else:
        raise ValueError("bisection failed to reach declared arithmetic resolution")
    repeats={name:[actual(f,pair,reverse=reverse,idle=idle) for reverse,idle in ((False,False),(False,False),(True,False),(True,True))]
        for name,pair in (("accepted",lp),("rejected",hp))}
    calculations={name:[probe(row["scale"],False)[0] for _ in range(2)] for name,row in (("accepted",low),("rejected",high))}
    return dict(label=label,largest_tested_accepted=low,smallest_tested_rejected=high,
        scale_interval=[low["scale"],high["scale"]],width_T=high["bias_magnitude_T"]-low["bias_magnitude_T"],
        relative_bracket_width=(high["scale"]-low["scale"])/((high["scale"]+low["scale"])/2),
        geometry_only_midpoint=mid,repeat_decisions=repeats,repeat_calculations=calculations,trace=trace)


def validate():
    f=fixture(); g=geometry(f)
    rows=[]
    zero,_=evaluate(f,g,np.zeros(3),"zero"); rows.append(zero)
    original,_=evaluate(f,g,np.array([1e-6,-2e-6,3e-6]),"original_8B2A"); rows.append(original)
    for fraction in (-.9,-.5,.1,1.,10.):
        row,_=evaluate(f,g,fraction*f["body_T"],f"parallel_{fraction}"); rows.append(row)
    for axis in ("in_plane","cross_plane"):
        for sign in (-1,1):
            for fraction in (1e-9,1e-8,1e-7,1e-5,1e-4,1e-3):
                row,_=evaluate(f,g,sign*fraction*g["field_magnitude_T"]*g[axis+"_B"],f"{axis}_{sign}_{fraction}"); rows.append(row)
    boundaries={axis+suffix:boundary(f,g,sign*g[axis+"_B"],axis+suffix,scale_T=g["field_magnitude_T"])
        for axis in ("in_plane","cross_plane") for sign,suffix in ((1,"_positive"),(-1,"_negative"))}
    original_vector=np.array([1e-6,-2e-6,3e-6])
    boundaries["original_direction"]=boundary(f,g,original_vector/np.linalg.norm(original_vector),"lambda_times_[1,-2,3]_uT",
        scale_T=float(np.linalg.norm(original_vector)))
    edges={name:evaluate(f,g,multiplier*f["body_T"],name)[0] for name,multiplier in (("zero_field",-1.),("polarity_reversal",-2.))}
    tested=rows+[r for b in boundaries.values() for r in b["trace"]]
    tolerance=f["policy"]["acquisition_pair_tolerance"]["value"]
    matrix=np.column_stack([g["parallel_B"],g["in_plane_B"],g["cross_plane_B"]])
    formulas=[]
    for row in tested:
        # Independent geometric basis formula: bias/|B| = a*ep + b*ei + d*ec.
        a,b,d=matrix.T @ np.array(row["bias_B_T"])/g["field_magnitude_T"]
        cos_measured=(g["cosine"]*(1+a)+g["sine"]*b)/math.sqrt((1+a)**2+b*b+d*d)
        metric=abs(cos_measured-zero["independent"]["reference_cosine"])
        angle=math.atan2(math.hypot(b,d),1+a)
        formulas.append(dict(label=row["label"],formula_metric=metric,formula_B_angle_rad=angle,
            metric_discrepancy=abs(metric-row["independent"]["metric"]),
            angle_discrepancy_rad=abs(angle-row["magnetic_direction_change_rad"])))
    checks=dict(
        independent_predicate_matches_all_actual_acquisitions=all(r["agrees"] for r in tested),
        zero_bias_accepts=zero["actual"]["decision"]=="ACCEPT",
        original_bias_rejection_reconstructed=original["actual"]["decision"]=="REJECT"
            and abs(original["independent"]["metric"]-f["original_gap"])<=NUMERIC_MARGIN,
        geometric_basis_orthonormal_right_handed=np.allclose(matrix.T @ matrix,np.eye(3),rtol=0,atol=NUMERIC_MARGIN)
            and abs(np.linalg.det(matrix)-1.)<=NUMERIC_MARGIN,
        parallel_magnitude_only_cases_accept=all(r["actual"]["decision"]=="ACCEPT" and r["independent"]["metric"]<NUMERIC_MARGIN
            for r in rows if r["label"].startswith("parallel_")),
        perpendicular_signed_cases_show_both_decisions=all({r["actual"]["decision"] for r in rows
            if r["label"].startswith(f"{axis}_{sign}_")}=={"ACCEPT","REJECT"} for axis in ("in_plane","cross_plane") for sign in (-1,1)),
        all_boundaries_bracket_unchanged_tolerance=all(b["largest_tested_accepted"]["independent"]["metric"]<=tolerance
            <b["smallest_tested_rejected"]["independent"]["metric"] and b["width_T"]>0 for b in boundaries.values()),
        all_endpoint_decisions_repeat_across_order_and_idle_history=all(all(x==values[0] for x in values)
            for b in boundaries.values() for values in b["repeat_decisions"].values()),
        endpoint_measurement_and_independent_calculation_repeat_exactly=all(
            all(r[key]==b[side][key] for r in values for key in ("measurement_S_T","configuration_fingerprint","independent","magnetic_direction_change_rad"))
            for b in boundaries.values() for name,side in (("accepted","largest_tested_accepted"),("rejected","smallest_tested_rejected"))
            for values in (b["repeat_calculations"][name],)),
        raw_metric_and_vector_angle_match_basis_geometry=max(r["metric_discrepancy"] for r in formulas)<NUMERIC_MARGIN
            and max(r["angle_discrepancy_rad"] for r in formulas)<NUMERIC_MARGIN,
        parallel_zero_and_polarity_reversal_are_separate_rejections=edges["zero_field"]["agrees"] and edges["polarity_reversal"]["agrees"]
            and edges["zero_field"]["independent"]["reason"]=="finite nonzero vector required"
            and edges["polarity_reversal"]["independent"]["reason"]=="inconsistent_acquisition_pair",
        all_finite_values_and_quiet_sensor_validity_preserved=all(r["sensor_valid"] and not r["sensor_reasons"]
            and np.isfinite(r["measurement_S_T"]).all() and r["actual"]["input_validity_and_values_unchanged"]
            and all(r["actual"]["sensor_valid"].values()) and r["acquisition_ns"]==r["publication_ns"]==400_000_000 for r in tested),
        binary64_predicate_error_within_declared_arithmetic_band=max(abs(r["binary64_minus_independent"]) for r in tested)<NUMERIC_MARGIN,
        runtime_models_gate_Q_R_P0_unchanged=subprocess.run(["git","diff","--quiet","HEAD","--",*("basilisk_runner/"+n
            for n in f["source"]["source_sha256"])],cwd=ROOT).returncode==0)
    sources=("validate_acquisition_boundary.py","test_acquisition_boundary.py","attitude_mekf.py","attitude_mekf_adapter.py",
        "tam_sensor_model.py","attitude_mekf_prototype.py","config/attitude_mekf_test_only.json")
    return dict(scope=SCOPE,passed=bool(all(checks.values())),engineering_gate="PASS" if all(checks.values()) else "INVESTIGATE",
        checks={k:bool(v) for k,v in checks.items()},base_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        source_evidence=f["source"],source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        environment=dict(python=platform.python_version(),numpy=np.__version__),geometry=g,policy=f["policy"],
        original_pair=asdict(f["pair"]),quiet_context=vars(f["context"]),parallel_edge_cases=edges,
        perturbation_provenance=dict(status="ASSUMED",source="validate_acquisition_boundary.py",revision="2026-10-02",
            units="T; scale is dimensionless",frame="B directions converted explicitly to S by C_SB",runtime_usable_for_flight=False),
        acquisition_tolerance=tolerance,numerical_margin=NUMERIC_MARGIN,rows=rows,boundaries=boundaries,
        angular_interpretation=dict(basis_formula="c_m=(c*(1+a)+sin(theta)*b)/sqrt((1+a)^2+b^2+d^2); bias/|B|=a*ep+b*ei+d*ec",
            angle_formula="delta_B=atan2(hypot(b,d),1+a); gate=abs(c_m-c_reference)",
            maximum_metric_formula_discrepancy=max(r["metric_discrepancy"] for r in formulas),
            maximum_angle_formula_discrepancy_rad=max(r["angle_discrepancy_rad"] for r in formulas)),
        tested_count=len(tested),maximum_binary64_metric_discrepancy=max(abs(r["binary64_minus_independent"]) for r in tested))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report",type=Path,default=HERE/"output_data/phase8b2b_acquisition_boundary.json")
    args=parser.parse_args(); report=validate()
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,default=json_value,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(dict(passed=report["passed"],checks=report["checks"],geometry=report["geometry"],
        boundaries={k:{side:{key:v[side][key] for key in ("scale","bias_magnitude_T","fraction_of_local_field","magnetic_direction_change_rad","independent","actual")}
            for side in ("largest_tested_accepted","smallest_tested_rejected")}
            for k,v in report["boundaries"].items()},maximum_binary64_metric_discrepancy=report["maximum_binary64_metric_discrepancy"]),default=json_value,indent=2))
    return 0 if report["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
