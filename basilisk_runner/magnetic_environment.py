"""Phase 2B Earth orientation and WMM input contract for Basilisk 2.10.2.

CONFIRMED implementation sources (v2.10.2, released 2026-05-08; wheel interfaces
and runtime probes checked against this exact release, not 2.11):
https://github.com/AVSLab/basilisk/blob/v2.10.2/src/simulation/environment/_GeneralModuleFiles/magneticFieldBase.cpp
https://github.com/AVSLab/basilisk/blob/v2.10.2/src/simulation/environment/magneticFieldWMM/magneticFieldWMM.cpp
Base uses r_BN_N - planet.PositionVector [m], then J20002Pfix = C_PN.
WMM evaluates geocentric spherical coordinates and returns magField_N [T].
Missing planet input defaults to origin/identity; input ages are not checked.
Output header is UpdateState's time, not the input state's time. The WMM
secular-variation clock rounds elapsed time to whole seconds internally.

ASSUMED environment fidelity: N is Earth-centered ICRF/J2000; P approximates
Earth-fixed using the IAU_EARTH model in NAIF pck00011 (2022-12-27), Earth
section: RA = -0.641 T deg; DEC = 90 - 0.557 T deg; W = 190.147 +
360.9856235 d deg. T is Julian centuries and d is days since J2000 TDB.
C_PN = R3(W) R1(pi/2-DEC) R3(pi/2+RA), all passive rotations.
https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/pck00011.tpc
https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/pck.html
This is NOT ITRF/EOP precision: NAIF documents >=150 arcsec prime-meridian
error for IAU_EARTH. No measured UT1/polar-motion/nutation corrections here.

The unchanged 2026.0 assumed epoch is explicitly 2026-01-01 00:00:00 UTC.
UTC -> TDB uses the NAIF naif0012 (2016-07-14) DELTET approximation and
TAI-UTC = 37 s, applicable on this date. No SPICE runtime or kernels required.
https://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/naif0012.tls
This helper is intentionally restricted to the current 2026 one-orbit case;
a new epoch/long run needs a reviewed leap-second/environment configuration.
"""

from datetime import datetime, timezone
import math

import numpy as np
from Basilisk.architecture import messaging, sysModel
from hs2_sim_config import DEFAULT_CONFIG

MODEL_NAME = DEFAULT_CONFIG.environment.earth_orientation_model.value
EPOCH_UTC = datetime.fromisoformat(DEFAULT_CONFIG.environment.epoch_utc.value)
J2000_CALENDAR = datetime(2000, 1, 1, 12, tzinfo=timezone.utc)
EPOCH_CALENDAR_SECONDS = (EPOCH_UTC - J2000_CALENDAR).total_seconds()


def tdb_seconds(time_s):
    """Seconds past J2000 TDB; NAIF DELTET formula, 2026 leap-second baseline."""
    tt = EPOCH_CALENDAR_SECONDS + np.asarray(time_s) + 37.0 + 32.184
    et = tt
    for _ in range(3):
        mean_anomaly = 6.239996 + 1.99096871e-7 * et
        et = tt + 1.657e-3 * np.sin(mean_anomaly + 1.671e-2 * np.sin(mean_anomaly))
    return et


def earth_dcm(et_s):
    """Return C_PN and dC_PN/d(TDB seconds), with the source conventions above."""
    days = float(et_s) / 86400.0
    centuries = days / 36525.0
    angles = np.radians([(190.147 + 360.9856235 * days) % 360.0,
                         0.557 * centuries, 90.0 - 0.641 * centuries])
    rates = np.radians([360.9856235 / 86400.0,
                        0.557 / (36525.0 * 86400.0), -0.641 / (36525.0 * 86400.0)])
    rotations, derivatives = [], []
    for axis, angle, rate in zip((3, 1, 3), angles, rates):
        c, s = math.cos(angle), math.sin(angle)
        if axis == 3:
            rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            derivative = rate * np.array([[-s, c, 0], [-c, -s, 0], [0, 0, 0]])
        else:
            rotation = np.array([[1, 0, 0], [0, c, s], [0, -s, c]])
            derivative = rate * np.array([[0, 0, 0], [0, -s, c], [0, -c, -s]])
        rotations.append(rotation)
        derivatives.append(derivative)
    a, b, c = rotations
    da, db, dc = derivatives
    return a @ b @ c, da @ b @ c + a @ db @ c + a @ b @ dc


class EarthOrientation(sysModel.SysModel):
    """Publish the supported SpicePlanetStateMsg interface, Earth at N's origin.

    PlanetEphemeris also supplies this message, but requires heliocentric
    Kepler elements and would move Earth's origin. This orientation-only
    publisher preserves the existing Earth-centered spacecraft orbit.
    """
    def __init__(self, epoch_fractional_year=DEFAULT_CONFIG.environment.epoch_fractional_year.value):
        super().__init__()
        if epoch_fractional_year != 2026.0:
            raise ValueError("Review the orientation/time baseline before changing the 2026.0 epoch")
        self.planetOutMsg = messaging.SpicePlanetStateMsg()

    def UpdateState(self, current_ns):
        time_s = current_ns * 1e-9
        if not 0.0 <= time_s < 86400.0:
            raise ValueError("Earth orientation baseline is limited to the first UTC day of 2026")
        et = float(tdb_seconds(time_s))
        matrix, derivative = earth_dcm(et)
        # DELTET derivative converts the analytic matrix rate to simulation SI seconds.
        mean_anomaly = 6.239996 + 1.99096871e-7 * et
        derivative *= 1.0 / (1.0 - 1.657e-3 * math.cos(mean_anomaly + 1.671e-2 * math.sin(mean_anomaly))
                             * 1.99096871e-7 * (1.0 + 1.671e-2 * math.cos(mean_anomaly)))
        payload = messaging.SpicePlanetStateMsgPayload()
        payload.PlanetName = "earth"
        payload.PositionVector = [0.0, 0.0, 0.0]
        payload.VelocityVector = [0.0, 0.0, 0.0]
        payload.J2000Current = et
        payload.J20002Pfix = matrix.tolist()
        payload.J20002Pfix_dot = derivative.tolist()
        payload.computeOrient = True
        self.planetOutMsg.write(payload, current_ns, self.moduleID)


class WMMInputGuard(sysModel.SysModel):
    """Check the actual WMM subscribers immediately before WMM UpdateState."""
    def __init__(self, wmm):
        super().__init__()
        self.wmm = wmm
        self.history = []

    def Reset(self, current_ns):
        self.history.clear()

    def UpdateState(self, current_ns):
        state = self.wmm.scStateInMsgs[0]
        earth = self.wmm.planetPosInMsg
        for name, reader in (("WMM spacecraft position", state), ("Earth orientation", earth)):
            if not reader.isLinked() or not reader.isWritten() or reader.timeWritten() != current_ns:
                raise ValueError(f"Missing/stale {name} at task tick {current_ns}")
        planet = earth()
        matrix = np.asarray(planet.J20002Pfix)
        if (not planet.computeOrient or not np.isfinite(matrix).all() or not math.isfinite(planet.J2000Current)
                or not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-12, rtol=0)
                or abs(np.linalg.det(matrix) - 1.0) > 1e-12
                or abs(planet.J2000Current - float(tdb_seconds(current_ns * 1e-9))) > 1e-6):
            raise ValueError("Invalid Earth orientation or inconsistent absolute epoch")
        self.history.append({"time_ns": current_ns, "wmm_state_time_ns": state.timeWritten(),
                             "earth_orientation_time_ns": earth.timeWritten()})
