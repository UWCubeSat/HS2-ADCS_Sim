# Validation plan

## Principle

Do not compare the old standalone 70-column CSV against Basilisk column-for-column. Basilisk produces message logs with different schema. Compare equivalent physical behavior.

## Canonical Basilisk output

The Basilisk runner writes canonical CSV files under:

```text
basilisk_runner/output_data/
```

Minimum useful columns:

```text
time_s
r_N_x_m, r_N_y_m, r_N_z_m
v_N_x_m_s, v_N_y_m_s, v_N_z_m_s
sigma_BN_1, sigma_BN_2, sigma_BN_3
omega_B_x_rad_s, omega_B_y_rad_s, omega_B_z_rad_s
omega_mag_rad_s
```

Detumble/magnetic columns:

```text
B_N_x_T, B_N_y_T, B_N_z_T, B_N_mag_T
B_B_x_T, B_B_y_T, B_B_z_T, B_B_mag_T
mcmd_x_Am2, mcmd_y_Am2, mcmd_z_Am2
ix_A, iy_A, iz_A
pcoil_x_W, pcoil_y_W, pcoil_z_W, pcoil_total_W
saturation_x, saturation_y, saturation_z
controller_valid
```

Disturbance/pointing columns are optional until implemented.

## Metrics

| Metric | Reference source | Basilisk source | Pass/fail logic |
|---|---|---|---|
| Initial angular speed | `sqrt(p^2+q^2+r^2)` at first reference row | `omega_mag_rad_s` first row | Should match configured initial condition closely. |
| Final angular speed | final reference row | final Basilisk row | Compare trend first; exact equality not expected. |
| Detumble time | first time below threshold | first time below threshold | Use thresholds such as 0.05, 0.02, 0.01 rad/s. |
| Body-rate decay | time history of `p,q,r` | time history of `omega_B_*` | Compare monotonic/trend and final magnitude. |
| Magnetic field magnitude | `sqrt(BBx^2+BBy^2+BBz^2)` | `B_B_mag_T` or `B_N_mag_T` | Should be same order and similar orbital trend after frame/epoch check. |
| Commanded dipole | `mcmd_*` | `mcmd_*_Am2` | Saturation behavior should match controller logic. |
| Commanded current | `ix,iy,iz` | `ix_A,iy_A,iz_A` | Verify current limits. |
| Coil power | `pcoil_total` | `pcoil_total_W` | Mean/peak should be comparable after controller match. |
| Disturbance torque | `Tdist_*` | disturbance columns | Only compare after disturbances are implemented. |
| Numerical sanity | all numeric columns | all numeric columns | No NaNs or infinities. |

## Initial reference metrics from uploaded standalone run

Known-good standalone summary:

```text
Rows: 57943
Cols: 70
All numeric values finite?: True
Initial |w| [rad/s]: 0.8774964387392122
Final   |w| [rad/s]: 0.010041780947787082
Mean coil power [W]: 0.7946818911022777
Peak coil power [W]: 2.5212665406427224
Mean drag force [N]: 2.0595744595556306e-07
Mean SRP force [N]: 9.865491785096374e-08
Mean total disturbance torque [N m]: 1.2038253703098527e-07
```

## Test sequence

1. Run `scenario_huskysat2_minimal.py` and confirm no NaN/Inf.
2. Run `scenario_huskysat2_detumble.py` with WMM and magnetometer but controller gain set to zero. Confirm field logging and no dynamics instability.
3. Enable Python detumble controller. Confirm rates decrease.
4. Build optional pybind C++ core. Confirm Python and C++ controller outputs match for the same input vectors.
5. Compare against standalone metrics.
6. Add disturbances only after the magnetic detumble loop is stable.
7. Re-run comparison after each subsystem is added.
