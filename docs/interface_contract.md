# ADCS interface contract

This contract defines the clean boundary between Basilisk and the project-specific ADCS core.

The goal is that `cpp_adcs_core` does **not** depend on Basilisk. Python is responsible for converting Basilisk messages into this interface.

## Function shape

Conceptual Python-side call:

```python
cmd = adcs_core.step(
    time=t,
    r_N=r_N,
    v_N=v_N,
    attitude=attitude,
    omega_B=omega_B,
    mag_B=mag_B,
    sun_vec_B=sun_vec_B,
    mode=mode,
)
```

The current C++ scaffold focuses on detumble and exposes the minimum useful subset:

```text
omega_B_rad_s
mag_B_T
```

The larger interface below is the intended stable contract.

## Inputs

| Name | Type | Units | Frame | Description | Status |
|---|---:|---:|---|---|---|
| `time_s` | scalar | s | simulation clock | Time since scenario start. | Required |
| `r_N_m` | length-3 vector | m | inertial N | Spacecraft position relative to central body. | Planned |
| `v_N_m_s` | length-3 vector | m/s | inertial N | Spacecraft velocity relative to central body. | Planned |
| `attitude_type` | string | - | - | Attitude representation identifier, e.g. `MRP_BN` or `quat_IB`. | Required for full interface |
| `sigma_BN` | length-3 vector | dimensionless | B relative to N | Basilisk MRP attitude. | Preferred native Basilisk input |
| `q_IB` | length-4 vector | dimensionless | standalone convention | Scalar-first quaternion from standalone if needed. | Conversion-only |
| `omega_BN_B_rad_s` | length-3 vector | rad/s | body B | Body angular velocity relative to inertial frame, expressed in body frame. | Required |
| `mag_B_T` | length-3 vector | T | body B / sensor S if aligned | Magnetic field expressed in body frame. | Required |
| `sun_vec_B` | length-3 vector | unitless | body B | Unit vector from spacecraft to Sun, expressed in body frame. | Planned |
| `mode` | enum/string | - | - | Controller mode, e.g. `DETUMBLE`, `POINTING`, `SAFE`, `OFF`. | Planned |
| `config` | struct/dict | mixed | - | Controller gains, actuator limits, coil properties. | Required |

## Outputs

| Name | Type | Units | Frame | Description |
|---|---:|---:|---|---|
| `commanded_magnetic_dipole_B_Am2` | length-3 vector | A m^2 | body B / MTB array basis | Commanded magnetic dipole after saturation. |
| `commanded_coil_current_A` | length-3 vector | A | actuator axes | Coil current command after current and dipole limiting. |
| `commanded_control_torque_B_Nm` | length-3 vector | N m | body B | Diagnostic torque estimate `m x B`; Basilisk `MtbEffector` should apply the truth torque. |
| `coil_power_W` | length-3 vector | W | per axis | `I^2 R` coil power estimate. |
| `coil_power_total_W` | scalar | W | - | Sum of per-axis coil power. |
| `controller_mode` | string/int | - | - | Active controller mode. |
| `saturation_flags` | length-3 bool vector | - | per axis | True where command was saturated. |
| `valid` | bool | - | - | False if input was invalid or magnetic field was degenerate. |
| `diagnostics` | dict | mixed | - | Optional B-dot, raw dipole, gains, errors, etc. |

## Units

Use SI units only:

- time: seconds
- distance: meters
- velocity: meters/second
- angular rate: radians/second
- magnetic field: Tesla
- dipole: A m^2
- current: A
- torque: N m
- power: W

## Frames and ordering

Default vector ordering is always:

```text
[x, y, z]
```

Basilisk-side native states:

```text
r_BN_N      position, inertial N
v_BN_N      velocity, inertial N
sigma_BN    MRP attitude of body B relative to inertial N
omega_BN_B  angular velocity of B relative to N, expressed in B
```

Standalone-side reference states:

```text
x,y,z, xdot,ydot,zdot, q0,q1,q2,q3, p,q,r
```

TODO: confirm exact standalone quaternion label convention before using `q_IB` for any control calculation.

## Magnetic field convention

Preferred controller input:

```text
mag_B_T = magnetic field vector expressed in body frame B, Tesla
```

In the Basilisk detumble scenario, this comes from the magnetometer module with sensor frame aligned to body frame:

```text
dcm_SB = identity
```

TODO: confirm whether Basilisk WMM `magField_N` sign and frame match the standalone `BfieldReference_eci_T` output.

## Sign convention for magnetorquer torque

Diagnostic torque convention:

```text
torque_B_Nm = commanded_magnetic_dipole_B_Am2 x mag_B_T
```

Basilisk `MtbEffector` should be treated as the truth torque application path. The adapter's torque is for logging and sanity checks.

TODO: confirm Basilisk `MtbEffector` internal sign against a one-step controlled test.

## Saturation convention

For each axis:

```text
allowed_current = min(current_limit_A, dipole_limit_Am2 / dipole_gain_Am2_per_A)
raw_current = desired_dipole_Am2 / dipole_gain_Am2_per_A
commanded_current = clamp(raw_current, -allowed_current, +allowed_current)
commanded_dipole = commanded_current * dipole_gain
```

If a dipole gain or limit is nonpositive, command zero on that axis and set saturation/invalid diagnostic.

## Mode convention

Initial enum/string set:

| Mode | Meaning |
|---|---|
| `OFF` | Zero actuator command. |
| `DETUMBLE` | Rate damping using magnetic field and body rate. |
| `POINTING` | Attitude/pointing control; not active in this scaffold. |
| `SAFE` | Conservative low-power or fault mode; not active in this scaffold. |

TODO: replace with the actual HuskySat mode table.

## Error handling

The ADCS core should return a zero command and `valid=False` when:

- any required vector is NaN or Inf
- magnetic field magnitude is below threshold
- required config values are invalid
- mode is unknown

Python adapter should not crash a long Basilisk run for a single invalid controller input unless the failure indicates a scenario setup error.

## Required TODOs before claiming equivalence

```python
# TODO: replace with actual HuskySat inertia
# TODO: verify magnetorquer coil parameters
# TODO: confirm magnetic-field frame convention
# TODO: match initial angular velocity to standalone reference
# TODO: match orbit to standalone reference
# TODO: confirm whether Basilisk WMM output matches standalone WMM convention
# TODO: confirm Basilisk MtbEffector torque sign with one-step test
# TODO: decide whether standalone navigation scaffold should be replaced by simpleNav or a custom estimator
```
