"""First-party runtime inputs for the Phase 3 detumble model, centralized in Phase 4.

Defaults preserve c1b6021 (2026-09-05). This is a runtime baseline, not a released
HS-2 vehicle configuration. Requirements/candidate values stay in
docs/REQUIREMENTS_BASELINE.md and docs/PHYSICAL_PARAMETERS.md. In particular,
the 3.72911 kg budget estimate is NOT selected; actual COM/inertia remain TBD.

Use dataclasses.replace to make explicit, provenance-bearing configurations.
No environment variables, requirements-derived substitutions or silent fallback.
Scientific model coefficients, message ABI sizes, mathematical constants and
validation tolerances remain in their implementations, not editable inputs here.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from typing import Any, cast

import numpy as np

STATUSES = frozenset({"CONFIRMED", "TBR", "TBC", "TBD", "ASSUMED"})
BASELINE_REVISION = "c1b6021 / 2026-09-05"
PHYSICAL_SOURCE = "docs/PHYSICAL_PARAMETERS.md; Phase 3 scenario/adapter"
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


@dataclass(frozen=True)
class Parameter:
    value: Any
    units: str
    status: str
    source: str
    revision: str
    frame: str
    notes: str = ""

    def __post_init__(self):
        if not isinstance(self.status, str) or self.status not in STATUSES:
            raise ValueError(f"Unsupported engineering status: {self.status!r}")
        for name in ("units", "source", "revision", "frame"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"Parameter requires nonempty {name}")
        if not isinstance(self.notes, str):
            raise ValueError("Parameter notes must be immutable text")

        def check(value):
            if isinstance(value, tuple):
                for component in value:
                    check(component)
            elif isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError("Configuration numbers must be finite")
            elif value is not None and not isinstance(value, (str, bool)):
                raise ValueError("Parameter values must be immutable scalars or tuples")
        check(self.value)


def assumed(value, units, frame="model", notes="", source=PHYSICAL_SOURCE):
    return Parameter(value, units, "ASSUMED", source, BASELINE_REVISION, frame, notes)


def implementation(value, notes):
    return Parameter(value, "1", "CONFIRMED", "Phase 3 active software configuration",
                     BASELINE_REVISION, "software", notes)


_MASS = assumed(2.6, "kg", "spacecraft", "Legacy box; budget estimate is not released mass.")
_DIMENSIONS = assumed((0.1, 0.1, 0.2), "m", "B xyz", "Legacy 2U-sized box; documented HS-2 3U configuration conflicts.")
_IX, _IY, _IZ = tuple((_MASS.value / 12.0) * sum(_DIMENSIONS.value[j]**2 for j in range(3) if j != i)
                      for i in range(3))


@dataclass(frozen=True)
class SpacecraftConfig:
    mass: Parameter = _MASS
    dimensions: Parameter = _DIMENSIONS
    inertia: Parameter = assumed(((_IX, 0.0, 0.0), (0.0, _IY, 0.0), (0.0, 0.0, _IZ)), "kg m^2", "B about COM",
                                 "Explicit runtime tensor derived from the legacy uniform box. Flight full tensor/products TBD; changing mass does not silently rescale this tensor.")
    com: Parameter = assumed((0.0, 0.0, 0.0), "m", "B: r_BcB_B", "Origin equals COM only in this model; flight COM TBD.")


@dataclass(frozen=True)
class OrbitConfig:
    altitude: Parameter = assumed(600e3, "m", "above orbit reference radius", "Mission 400-600 km is TBR; no replacement selected.")
    inclination: Parameter = assumed(56.0, "deg", "N equator", "Conflicts with mission 47.6-51.6 deg TBR range.")
    eccentricity: Parameter = assumed(0.0, "1", "Kepler orbit", "Current Cartesian initializer supports circular orbits only.")
    initial_state_convention: Parameter = assumed("circular_at_positive_N_x", "1", "N", "r_N=[R+h,0,0]; v_N=[0,v*cos(i),v*sin(i)], v=sqrt(mu/(R+h)). No released RAAN/phase.")


@dataclass(frozen=True)
class EnvironmentConfig:
    mu_earth: Parameter = assumed(3.986004418e14, "m^3/s^2", "Earth central gravity")
    orbit_reference_radius: Parameter = assumed(6.371e6, "m", "Earth orbit reference sphere", "Different from WMM's reference radius; retain both.")
    wmm_reference_radius: Parameter = assumed(6371.2e3, "m", "WMM reference sphere")
    epoch_utc: Parameter = assumed("2026-01-01T00:00:00+00:00", "UTC", "simulation epoch", "Reviewed Earth time helper supports only the first UTC day of 2026; mission epoch TBD.")
    magnetic_model: Parameter = implementation("WMM2025", "Configured software choice, not a mission geomagnetic-model requirement.")
    coefficient_filename: Parameter = implementation("WMM2025.COF", "Preserve installed-package-first then standalone-data search; resolved file is recorded at runtime.")
    earth_orientation_model: Parameter = implementation("IAU_EARTH_pck00011_low_order", "Phase 2B implementation selection. Physical fidelity ASSUMED; no measured EOP/UT1/polar motion.")
    gravity_model: Parameter = implementation("central_earth", "Single central Earth body; no additional gravity bodies.")
    disturbances: Parameter = assumed("none", "1", "B torque", "No drag, SRP, gravity-gradient or residual-dipole torque; omission does not establish zero flight disturbance.")

    @property
    def epoch_fractional_year(self):
        # Validation restricts the epoch to January 1; keep WMM's exact 2026.0.
        return assumed(float(datetime.fromisoformat(self.epoch_utc.value).year), "year", "WMM epoch",
                       "Derived from epoch_utc; no independent epoch selection.")


@dataclass(frozen=True)
class TimingConfig:
    dynamics_step: Parameter = assumed(0.1, "s", "simulation task clock", "RK4 propagation and following-interval command hold.")
    environment_step: Parameter = assumed(0.1, "s", "simulation task clock")
    sensor_step: Parameter = assumed(0.1, "s", "simulation task clock")
    control_step: Parameter = assumed(0.1, "s", "simulation task clock")
    record_step: Parameter = assumed(1.0, "s", "simulation task clock", "Exact-tick state/telemetry sampling; command replay can record every task tick.")
    duration_orbits: Parameter = assumed(1.0, "orbit", "scenario duration", "One period of the configured circular orbit; not a detumble requirement.")
    duration_override: Parameter = assumed(None, "s", "scenario duration", "None means duration_orbits times orbit period. Explicit test/CLI override is recorded here.")


@dataclass(frozen=True)
class MagnetorquerConfig:
    count: Parameter = assumed(3, "1", "actuator array", "Provisional three-axis model; no released mounting/BOM selection.")
    axes_B: Parameter = assumed(IDENTITY, "1", "B: columns are actuator axes", "Ideal body alignment; physical HS-2 axes/mounting TBD.")
    dipole_limits: Parameter = assumed((0.2, 0.2, 0.85), "A m^2", "actuator axes", "Symmetric native clipping. Z is legacy MT01-derived, not confirmed installed MT01A capability.")
    resistance: Parameter = assumed((51.0, 51.0, 4.4), "ohm", "actuator xyz", "Legacy constant resistance; no temperature model.")
    rod_voltage_limits: Parameter = assumed((5.0, 5.0), "V", "actuator xy", "Used only to derive rod current limits; no voltage/H-bridge dynamics.")
    rod_dipole_gains: Parameter = assumed((2.3, 2.3), "A m^2/A", "actuator xy", "Vendor-derived nominal gains, not installed calibration.")
    aircoil_power_limit: Parameter = assumed(1.75, "W", "actuator z", "Derives Z current/gain; conflicting ICD/budget/datasheet operating points remain unresolved.")
    implementation: Parameter = implementation("native", "MtbEffector default; direct is a controlled ExtForceTorque reference.")

    @property
    def current_limits(self):
        r = self.resistance.value
        voltage = self.rod_voltage_limits.value
        return assumed((voltage[0]/r[0], voltage[1]/r[1], math.sqrt(self.aircoil_power_limit.value/r[2])),
                       "A", "actuator xyz", "Derived [Vx/Rx,Vy/Ry,sqrt(Pz/Rz)]; dipole limit further clips current.")

    @property
    def dipole_gains(self):
        return assumed((*self.rod_dipole_gains.value, self.dipole_limits.value[2]/self.current_limits.value[2]),
                       "A m^2/A", "actuator xyz", "Z gain inferred from dipole/power limits; no measured calibration.")


@dataclass(frozen=True)
class SensorConfig:
    magnetometer_dcm_SB: Parameter = assumed(IDENTITY, "1", "C_SB: B to sensor S", "S=B in current controller; mounting calibration TBD.")
    magnetometer_scale: Parameter = assumed(1.0, "1", "sensor S")
    magnetometer_bias: Parameter = assumed((0.0, 0.0, 0.0), "T", "sensor S")
    magnetometer_noise_std: Parameter = assumed((0.0, 0.0, 0.0), "T", "sensor S", "Ideal TAM; no interference, quantization or flight calibration.")
    magnetometer_min_output: Parameter = assumed(-1e200, "T", "sensor S", "Unchanged Basilisk 2.10.2 effectively unbounded output default.")
    magnetometer_max_output: Parameter = assumed(1e200, "T", "sensor S", "Unchanged Basilisk 2.10.2 effectively unbounded output default.")
    navigation_model: Parameter = implementation("SimpleNav_truth", "Ideal spacecraft-truth navigation; no gyro or estimator model.")
    navigation_noise_matrix: Parameter = assumed(tuple((0.0,)*18 for _ in range(18)), "mixed SI", "SimpleNav 18-state order", "Zero PMatrix verified against installed 2.10.2; no stochastic navigation errors.")
    navigation_walk_bounds: Parameter = assumed((0.0,)*18, "mixed SI", "SimpleNav 18-state order", "Zero walkBounds; no random-walk navigation error.")


@dataclass(frozen=True)
class DetumbleControllerConfig:
    dipole_command_gain: Parameter = assumed(67200.0, "A m^2 s/(rad T)", "B", "m_desired=K*(omega_B cross B_B); unchanged controller law.")
    minimum_field: Parameter = assumed(1e-12, "T", "B field norm", "Controller validity threshold, not sensor accuracy.")
    use_cpp_core_if_available: Parameter = implementation(True, "Preserve optional adcs_core delegation and existing Python fallback policy.")


@dataclass(frozen=True)
class InitialConditionConfig:
    body_rate: Parameter = assumed((0.8, -0.2, 0.3), "rad/s", "omega_BN_B", "Legacy synthetic tumble; not a deployment-rate requirement.")
    sigma_BN: Parameter = assumed((0.0, 0.0, 0.0), "1", "MRP B relative N", "C_BN maps N components into B.")


# Every section is a dataclass with explicit Parameter defaults; none uses
# dataclasses.MISSING. Keep reflection at this narrowly typed schema boundary.
ConfigSection = (SpacecraftConfig | OrbitConfig | EnvironmentConfig | TimingConfig
                 | MagnetorquerConfig | SensorConfig | DetumbleControllerConfig
                 | InitialConditionConfig)


@dataclass(frozen=True)
class HS2SimConfig:
    spacecraft: SpacecraftConfig = SpacecraftConfig()
    orbit: OrbitConfig = OrbitConfig()
    environment: EnvironmentConfig = EnvironmentConfig()
    timing: TimingConfig = TimingConfig()
    magnetorquers: MagnetorquerConfig = MagnetorquerConfig()
    sensors: SensorConfig = SensorConfig()
    controller: DetumbleControllerConfig = DetumbleControllerConfig()
    initial: InitialConditionConfig = InitialConditionConfig()

    def __post_init__(self):
        self.validate()

    @property
    def orbit_period_s(self):
        r = self.environment.orbit_reference_radius.value + self.orbit.altitude.value
        return 2.0 * math.pi * math.sqrt(r**3 / self.environment.mu_earth.value)

    @property
    def duration_s(self):
        override = self.timing.duration_override.value
        return self.orbit_period_s * self.timing.duration_orbits.value if override is None else override

    def initial_orbit_state(self):
        r0 = self.environment.orbit_reference_radius.value + self.orbit.altitude.value
        v0 = math.sqrt(self.environment.mu_earth.value / r0)
        inc = math.radians(self.orbit.inclination.value)
        return [r0, 0.0, 0.0], [0.0, v0 * math.cos(inc), v0 * math.sin(inc)]

    def with_run_options(self, duration_s=None, actuator=None):
        timing, mtq = self.timing, self.magnetorquers
        if duration_s is not None:
            timing = replace(timing, duration_override=assumed(float(duration_s), "s", "scenario duration", "Explicit run/test/CLI duration override."))
        if actuator is not None and actuator != mtq.implementation.value:
            mtq = replace(mtq, implementation=implementation(actuator, "Explicit run/test/CLI actuator selection."))
        return replace(self, timing=timing, magnetorquers=mtq)

    def validate(self):
        def require(condition, message):
            if not condition:
                raise ValueError(message)

        def numeric(parameter: Parameter, shape: tuple[int, ...] = (), positive: bool = False):
            array = np.asarray(parameter.value)
            require(array.shape == shape and array.dtype.kind in "fiu" and np.isfinite(array).all(),
                    f"Expected finite numeric shape {shape} for {parameter.frame}")
            require(not positive or np.all(array > 0), f"Expected positive value for {parameter.frame}")
            return array.astype(float)

        for section_field in fields(self):
            section = getattr(self, section_field.name)
            require(type(section) is type(section_field.default), f"Invalid config section {section_field.name}")
            for f in fields(section):
                p = getattr(section, f.name)
                require(isinstance(p, Parameter), f"{section_field.name}.{f.name} requires Parameter metadata")
                p.__post_init__()
                default = cast(Parameter, f.default)
                require(p.units == default.units and p.frame == default.frame,
                        f"Units/frame contract mismatch for {section_field.name}.{f.name}")

        numeric(self.spacecraft.mass, positive=True)
        numeric(self.spacecraft.dimensions, (3,), positive=True)
        numeric(self.spacecraft.com, (3,))
        inertia = numeric(self.spacecraft.inertia, (3, 3))
        require(np.array_equal(inertia, inertia.T), "Inertia tensor must be symmetric")
        require(np.all(np.linalg.eigvalsh(inertia) > 0), "Inertia tensor must be positive definite")
        # Current independent telemetry verifier supports the same zero-COM hub.
        require(np.all(np.asarray(self.spacecraft.com.value) == 0), "Nonzero COM needs a reviewed plant/validation reference-point contract")
        numeric(self.environment.mu_earth, positive=True)
        numeric(self.environment.orbit_reference_radius, positive=True)
        numeric(self.environment.wmm_reference_radius, positive=True)
        numeric(self.orbit.altitude, positive=True)
        inc = numeric(self.orbit.inclination)
        e = numeric(self.orbit.eccentricity)
        require(0 <= inc <= 180 and 0 <= e < 1, "Invalid orbit inclination/eccentricity")
        require(e == 0 and self.orbit.initial_state_convention.value == "circular_at_positive_N_x",
                "Only the reviewed circular positive-N-x initializer is supported")
        for p, supported in ((self.environment.epoch_utc, "2026-01-01T00:00:00+00:00"),
                             (self.environment.magnetic_model, "WMM2025"),
                             (self.environment.coefficient_filename, "WMM2025.COF"),
                             (self.environment.earth_orientation_model, "IAU_EARTH_pck00011_low_order"),
                             (self.environment.gravity_model, "central_earth"),
                             (self.environment.disturbances, "none"),
                             (self.sensors.navigation_model, "SimpleNav_truth")):
            require(p.value == supported, f"Unsupported model/epoch selection: {p.value!r}; implementation review required")

        steps = [self.timing.dynamics_step, self.timing.environment_step, self.timing.sensor_step,
                 self.timing.control_step, self.timing.record_step]
        for p in steps:
            numeric(p, positive=True)
            require(round(p.value*1e9) >= 1 and abs(p.value*1e9-round(p.value*1e9)) < 1e-6,
                    "Timing must be representable as positive integer nanoseconds")
        require(all(p.value == steps[0].value for p in steps[1:4]), "Current schedule requires equal dynamics/environment/sensor/control steps")
        require(round(steps[4].value*1e9) % round(steps[0].value*1e9) == 0,
                "Record step must be an integer multiple of dynamics step")
        numeric(self.timing.duration_orbits, positive=True)
        if self.timing.duration_override.value is not None:
            numeric(self.timing.duration_override, positive=True)
        require(math.isfinite(self.duration_s) and 0 < self.duration_s < 86400,
                "Duration must remain within the reviewed first UTC day of 2026")

        mtq = self.magnetorquers
        require(type(mtq.count.value) is int and mtq.count.value == 3, "Current controller requires exactly three actuators")
        axes = numeric(mtq.axes_B, (3, mtq.count.value))
        require(np.allclose(np.linalg.norm(axes, axis=0), 1, atol=1e-12, rtol=0),
                "Actuator columns must be nonzero unit axes")
        require(np.array_equal(axes, IDENTITY), "Current body-dipole controller requires body-aligned axes; allocation for other layouts is not implemented")
        numeric(mtq.dipole_limits, (3,), positive=True)
        numeric(mtq.resistance, (3,), positive=True)
        numeric(mtq.rod_voltage_limits, (2,), positive=True)
        numeric(mtq.rod_dipole_gains, (2,), positive=True)
        numeric(mtq.aircoil_power_limit, positive=True)
        numeric(mtq.current_limits, (3,), positive=True)
        numeric(mtq.dipole_gains, (3,), positive=True)
        require(mtq.implementation.value in ("native", "direct"), "Unsupported actuator implementation")
        numeric(self.controller.dipole_command_gain, positive=True)
        numeric(self.controller.minimum_field, positive=True)
        require(type(self.controller.use_cpp_core_if_available.value) is bool, "C++ core selection must be bool")
        numeric(self.initial.body_rate, (3,))
        numeric(self.initial.sigma_BN, (3,))
        sensors = self.sensors
        require(np.array_equal(numeric(sensors.magnetometer_dcm_SB, (3, 3)), IDENTITY),
                "Current controller consumes S=B; a nonidentity sensor transform needs adapter changes")
        numeric(sensors.magnetometer_scale, positive=True)
        numeric(sensors.magnetometer_bias, (3,))
        require(sensors.magnetometer_scale.value == 1 and np.all(np.asarray(sensors.magnetometer_bias.value) == 0),
                "Current ideal-TAM validation requires scale 1 and bias 0; sensor realism needs a reviewed measurement contract")
        noise = numeric(sensors.magnetometer_noise_std, (3,))
        require(np.all(noise == 0), "Stochastic TAM needs a reviewed seed/noise validation contract")
        require(np.all(numeric(sensors.navigation_noise_matrix, (18, 18)) == 0)
                and np.all(numeric(sensors.navigation_walk_bounds, (18,)) == 0), "Only ideal navigation is supported in this recovery phase")
        numeric(sensors.magnetometer_min_output)
        numeric(sensors.magnetometer_max_output)
        require(sensors.magnetometer_min_output.value < sensors.magnetometer_max_output.value, "Invalid TAM output bounds")

    def to_dict(self):
        return asdict(self)

    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True, allow_nan=False).encode()).hexdigest()

    @classmethod
    def from_dict(cls, data):
        def immutable(value):
            return tuple(immutable(x) for x in value) if isinstance(value, list) else value
        if set(data) != {f.name for f in fields(cls)}:
            raise ValueError("Configuration must contain exactly the eight runtime sections")
        # JSON keys select heterogeneous sections; validate their exact keys and
        # Parameter metadata below before passing them to the dataclass constructor.
        sections: dict[str, Any] = {}
        for f in fields(cls):
            schema = type(cast(ConfigSection, f.default))
            if set(data[f.name]) != {p.name for p in fields(schema)}:
                raise ValueError(f"Missing/unknown parameter in {f.name}")
            try:
                sections[f.name] = schema(**{name: Parameter(**{**meta, "value": immutable(meta["value"])})
                                             for name, meta in data[f.name].items()})
            except (TypeError, KeyError) as exc:
                raise ValueError(f"Invalid parameter metadata in {f.name}: {exc}") from exc
        return cls(**sections)

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


DEFAULT_CONFIG = HS2SimConfig()
