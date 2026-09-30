"""Phase 7F-2A command equations only; fixtures/tolerances ASSUMED / TEST-ONLY.

2026-09-30. Reuse accepted snapshot fixtures; do not repeat input/fault gates.
The standalone production SysModel below has only synthetic input messages and
test-read output buffers. No spacecraft, effector or task is instantiated.
"""
import ast
from dataclasses import FrozenInstanceError, asdict, replace
from decimal import Decimal, localcontext
import inspect
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from Basilisk.architecture import messaging

import basilisk_adcs_adapter as production
import disconnected_detumble_math as development
from hs2_sim_config import DEFAULT_CONFIG
from test_control_input_snapshot import capture, magnetic, navigation


def accepted_snapshot(rate, field, mounting=None) -> development.ControlSnapshot:
    nav = navigation()
    nav.omega = list(rate)
    tam = replace(magnetic(), tam_S_T=tuple(field), reference_N_T=None, reference_epoch_ns=None,
                  provenance="Phase 7F-2A mathematical TAM fixture, 2026-09-30; ASSUMED / TEST-ONLY")
    if mounting is not None:
        tam = replace(tam, dcm_SB=mounting)
    result = capture(nav, tam)
    if not result.accepted or result.snapshot is None:
        raise AssertionError(result.rejection_reason)
    return result.snapshot


def fixtures():
    # rad/s and T in B, except mounted fixture: its magnetic input is in S.
    cases = {"zero_rate": ((0., 0., 0.), (0., 0., 3e-5)),
             "parallel": ((0., 0., 2.), (0., 0., 3e-5)),
             "perpendicular": ((0.1, 0., 0.), (0., 2e-5, 0.)),
             "arbitrary_unsaturated": ((0.02, -0.01, 0.03), (2e-5, -3e-5, 4e-5)),
             "mixed_saturation": ((0.8, -0.2, 0.3), (2e-5, -3e-5, 4e-5)),
             "all_saturated": ((2., -3., 4.), (3e-5, 2e-5, -1e-5))}
    for axis in range(3):
        for sign in (-1, 1):
            rate, field = [0.]*3, [0.]*3
            rate[(axis+1) % 3] = sign*2.
            field[(axis+2) % 3] = 3e-5
            cases[f"clip_{axis}_{sign:+d}"] = (tuple(rate), tuple(field))
    result = {name: accepted_snapshot(rate, field) for name, (rate, field) in cases.items()}
    result["mounted_sensor"] = accepted_snapshot((0.2, 0.1, -0.1), (2e-5, -3e-5, 4e-5),
                                                  ((0., 1., 0.), (-1., 0., 0.), (0., 0., 1.)))
    return result


def production_reference(snapshot):
    """Actual unchanged SysModel, equivalent inputs at the math boundary.

    The production wrapper assumes its tam_S payload is B-aligned. Feed the
    snapshot's already transformed B vector here, not a new sensor mounting.
    """
    nav, tam = messaging.NavAttMsg(), messaging.TAMSensorMsg()
    nav_payload, tam_payload = messaging.NavAttMsgPayload(), messaging.TAMSensorMsgPayload()
    nav_payload.sigma_BN = list(snapshot.sigma_BN)
    nav_payload.omega_BN_B = list(snapshot.omega_BN_B_rad_s)
    nav_payload.timeTag = snapshot.navigation_state_ns*1e-9
    tam_payload.tam_S = list(snapshot.tam_B_T)
    nav.write(nav_payload, snapshot.navigation_state_ns)
    tam.write(tam_payload, snapshot.tam_acquisition_ns)
    controller = production.PythonBdotMTQController(production.ADCSConfig.from_sim_config(DEFAULT_CONFIG))
    controller.navAttInMsg.subscribeTo(nav)
    controller.tamSensorInMsg.subscribeTo(tam)
    controller.Reset(0)
    controller.UpdateState(snapshot.window.evaluation_ns)
    # Test-only reader for the native publication header; no actuator subscriber.
    command_reader = messaging.MTBCmdMsgReader()
    command_reader.subscribeTo(controller.mtbCmdOutMsg)
    return {"history": dict(controller.history[-1]),
            "dipole": tuple(controller.mtbCmdOutMsg.read().mtbDipoleCmds[:3]),
            "torque": tuple(controller.cmdTorqueOutMsg.read().torqueRequestBody),
            "command_epoch_ns": int(command_reader.timeWritten()),
            "nav_epoch_ns": int(controller.navAttInMsg.timeWritten()),
            "tam_epoch_ns": int(controller.tamSensorInMsg.timeWritten())}


def independent_math(logged_record):
    """Independent scalar Decimal oracle from logged inputs and sourced config.

    No np.cross, clipping utility, controller helper or controller output is used.
    Current/dipole limits are derived from the logged Parameter values. Compute
    the symmetric dipole bounds first, then recover current: independent order
    from production's current-first clamp. Rounding tolerance is TEST-ONLY.
    """
    with localcontext() as context:
        context.prec = 50
        dec = lambda value: Decimal(str(value))
        snapshot = logged_record["snapshot"]
        config = json.loads(logged_record["configuration_json"])
        mtq = config["magnetorquers"]
        gain = dec(config["controller"]["dipole_command_gain"]["value"])
        w = [dec(value) for value in snapshot["omega_BN_B_rad_s"]]
        b = [dec(value) for value in snapshot["tam_B_T"]]
        requested = [gain*(w[1]*b[2]-w[2]*b[1]), gain*(w[2]*b[0]-w[0]*b[2]), gain*(w[0]*b[1]-w[1]*b[0])]
        resistances = [dec(v) for v in mtq["resistance"]["value"]]
        limits = [dec(v) for v in mtq["dipole_limits"]["value"]]
        volts = [dec(v) for v in mtq["rod_voltage_limits"]["value"]]
        currents = [volts[0]/resistances[0], volts[1]/resistances[1],
                    (dec(mtq["aircoil_power_limit"]["value"])/resistances[2]).sqrt()]
        factors = [dec(v) for v in mtq["rod_dipole_gains"]["value"]]+[limits[2]/currents[2]]
        bounds = [min(limits[i], currents[i]*factors[i]) for i in range(3)]
        dipole = [min(bounds[i], max(-bounds[i], requested[i])) for i in range(3)]
        torque = [dipole[1]*b[2]-dipole[2]*b[1], dipole[2]*b[0]-dipole[0]*b[2], dipole[0]*b[1]-dipole[1]*b[0]]
        return {"requested": tuple(map(float, requested)), "dipole": tuple(map(float, dipole)),
                "current": tuple(float(dipole[i]/factors[i]) for i in range(3)),
                "torque": tuple(map(float, torque)), "bounds": tuple(map(float, bounds)),
                "flags": tuple(abs(requested[i]) > bounds[i] for i in range(3))}


class DisconnectedCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshots = fixtures()
        cls.outputs = {name: development.evaluate_snapshot(snap, snap.window.evaluation_ns) for name, snap in cls.snapshots.items()}
        cls.reference = {name: production_reference(snap) for name, snap in cls.snapshots.items()}
        cls.independent = {name: independent_math(asdict(command)) for name, command in cls.outputs.items()}

    def test_actual_production_sysmodel_matches_inputs_commands_and_epochs(self):
        for name, output in self.outputs.items():
            with self.subTest(name=name):
                reference, snap = self.reference[name], output.snapshot
                row = reference["history"]
                np.testing.assert_array_equal([row[f"omega_B_{a}_rad_s"] for a in "xyz"], snap.omega_BN_B_rad_s)
                np.testing.assert_array_equal([row[f"B_B_{a}_T"] for a in "xyz"], snap.tam_B_T)
                np.testing.assert_array_equal(output.clipped_dipole_B_Am2, reference["dipole"])
                np.testing.assert_array_equal(output.predicted_torque_B_Nm, reference["torque"])
                np.testing.assert_array_equal(output.coil_current_A, [row[f"i{a}_A"] for a in "xyz"])
                self.assertEqual(output.saturation_flags, tuple(row[f"saturation_{a}"] for a in "xyz"))
                self.assertEqual(output.command_valid, row["controller_valid"])
                self.assertTrue(output.command_valid)
                self.assertEqual(reference["command_epoch_ns"], output.evaluation_epoch_ns)
                self.assertEqual(reference["nav_epoch_ns"], reference["tam_epoch_ns"])
                self.assertEqual(reference["nav_epoch_ns"], snap.navigation_state_ns)

    def test_independent_decimal_math_from_logged_snapshots(self):
        for name, output in self.outputs.items():
            oracle = self.independent[name]
            with self.subTest(name=name):
                # ASSUMED / TEST-ONLY roundoff tolerances, not physical accuracy.
                np.testing.assert_allclose(output.requested_dipole_B_Am2, oracle["requested"], rtol=1e-12, atol=1e-14)
                np.testing.assert_allclose(output.clipped_dipole_B_Am2, oracle["dipole"], rtol=1e-12, atol=1e-14)
                np.testing.assert_allclose(output.coil_current_A, oracle["current"], rtol=1e-12, atol=1e-14)
                np.testing.assert_allclose(output.predicted_torque_B_Nm, oracle["torque"], rtol=1e-12, atol=1e-18)
                self.assertEqual(output.saturation_flags, oracle["flags"])

    def test_zero_and_parallel_rate_produce_zero_request_dipole_and_torque(self):
        for name in ("zero_rate", "parallel"):
            output = self.outputs[name]
            for value in (output.requested_dipole_B_Am2, output.clipped_dipole_B_Am2, output.predicted_torque_B_Nm):
                np.testing.assert_array_equal(value, (0., 0., 0.))

    def test_perpendicular_geometry_has_correct_command_and_damping_sign(self):
        # +X rate, +Y field => +Z dipole, -X torque. SI hand calculation.
        output = self.outputs["perpendicular"]
        np.testing.assert_allclose(output.requested_dipole_B_Am2, (0., 0., 0.1344), atol=1e-15, rtol=0)
        np.testing.assert_allclose(output.predicted_torque_B_Nm, (-2.688e-6, 0., 0.), atol=1e-18, rtol=0)
        for case in self.outputs.values():
            self.assertLessEqual(float(np.dot(case.predicted_torque_B_Nm, case.snapshot.omega_BN_B_rad_s)), 1e-18)

    def test_positive_negative_clipping_on_every_axis(self):
        for axis in range(3):
            for sign in (-1, 1):
                name = f"clip_{axis}_{sign:+d}"
                output, oracle = self.outputs[name], self.independent[name]
                self.assertTrue(output.saturation_flags[axis])
                self.assertAlmostEqual(output.clipped_dipole_B_Am2[axis], sign*oracle["bounds"][axis], places=14)
                self.assertGreater(abs(output.requested_dipole_B_Am2[axis]), oracle["bounds"][axis])
            positive, negative = self.outputs[f"clip_{axis}_+1"], self.outputs[f"clip_{axis}_-1"]
            np.testing.assert_array_equal(positive.predicted_torque_B_Nm, -np.array(negative.predicted_torque_B_Nm))

    def test_unsaturated_mixed_and_all_saturated_commands(self):
        self.assertEqual(self.outputs["arbitrary_unsaturated"].saturation_flags, (False, False, False))
        self.assertEqual(self.outputs["mixed_saturation"].saturation_flags, (False, True, True))
        self.assertEqual(self.outputs["all_saturated"].saturation_flags, (True, True, True))
        output = self.outputs["arbitrary_unsaturated"]
        np.testing.assert_allclose(output.requested_dipole_B_Am2, output.clipped_dipole_B_Am2, atol=1e-15, rtol=0)

    def test_snapshot_body_field_is_used_instead_of_raw_sensor_components(self):
        output = self.outputs["mounted_sensor"]
        self.assertNotEqual(output.snapshot.tam_S_T, output.snapshot.tam_B_T)
        np.testing.assert_array_equal(output.snapshot.tam_B_T, (3e-5, 2e-5, 4e-5))
        np.testing.assert_allclose(output.requested_dipole_B_Am2, (0.4032, -0.7392, 0.0672), atol=1e-14, rtol=0)

    def test_actual_dispatcher_receives_frozen_values_and_evaluation_time(self):
        snapshot = self.snapshots["perpendicular"]
        with patch.object(production, "controller_step", wraps=production.controller_step) as call:
            output = development.evaluate_snapshot(snapshot, snapshot.window.evaluation_ns)
        call.assert_called_once()
        time_s, rate, field, cfg = call.call_args.args
        self.assertEqual(time_s, 0.5)
        self.assertEqual(snapshot.navigation_state_ns, 400_000_000)
        self.assertEqual(rate, snapshot.omega_BN_B_rad_s)
        self.assertEqual(field, snapshot.tam_B_T)
        self.assertEqual(cfg, production.ADCSConfig.from_sim_config(DEFAULT_CONFIG))
        self.assertEqual(output.configuration_fingerprint, DEFAULT_CONFIG.fingerprint())
        self.assertEqual(json.loads(output.configuration_json), json.loads(json.dumps(DEFAULT_CONFIG.to_dict())))

    def test_record_has_no_native_command_endpoint_or_runtime_connection(self):
        tree = ast.parse(inspect.getsource(development))
        direct_imports = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        self.assertEqual(direct_imports, {"json", "basilisk_adcs_adapter"})
        attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertTrue({"write", "subscribeTo", "AddModelToTask", "addDynamicEffector", "PythonBdotMTQController"}.isdisjoint(attributes))
        output = self.outputs["perpendicular"]
        self.assertFalse(output.actuator_authority)
        self.assertFalse(hasattr(output, "mtbCmdOutMsg"))
        self.assertFalse(hasattr(output, "cmdTorqueOutMsg"))
        with self.assertRaises(FrozenInstanceError):
            setattr(output, "command_valid", False)
        runner = Path(__file__).resolve().parent
        for name in ("scenario_huskysat2_detumble.py", "magnetic_control_cycle.py", "basilisk_adcs_adapter.py", "attitude_mekf_adapter.py"):
            self.assertNotIn("disconnected_detumble_math", (runner/name).read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        fields = {"requested_dipole_B_Am2": "requested", "clipped_dipole_B_Am2": "dipole",
                  "coil_current_A": "current", "predicted_torque_B_Nm": "torque"}
        errors = {field: max(float(np.max(np.abs(np.array(getattr(value, field))-cls.independent[name][key])))
                            for name, value in cls.outputs.items()) for field, key in fields.items()}
        print("Phase 7F-2A measurements: "+json.dumps({"controlled_cases": len(cls.outputs),
            "production_max_dipole_error_Am2": max(float(np.max(np.abs(np.array(value.clipped_dipole_B_Am2)-cls.reference[name]["dipole"]))) for name, value in cls.outputs.items()),
            "production_max_torque_error_Nm": max(float(np.max(np.abs(np.array(value.predicted_torque_B_Nm)-cls.reference[name]["torque"]))) for name, value in cls.outputs.items()),
            "independent_max_abs_errors": errors, "optional_core_available": production._adcs_core is not None}, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
