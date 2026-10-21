import math
import unittest

from powertrain.engine import Engine
from core.magic_formulas import torque_curve




def make_engine(**overrides: float) -> Engine:
    settings = {
        "peak_torque": 400.0,
        "base_torque": 100.0,
        "peak_torque_rpm": 4500.0,
        "rise_width": 1800.0,
        "fall_width": 2500.0,
        "idle_rpm": 850.0,
        "redline_rpm": 7000.0,
        "inertia": 0.25,
    }
    settings.update(overrides)
    return Engine(**settings)




class TestTorqueCurve(unittest.TestCase):
    def test_torque_equals_peak_at_peak_rpm(self) -> None:
        torque = torque_curve(
            rpm=4500.0,
            peak_torque=400.0,
            base_torque=100.0,
            peak_rpm=4500.0,
            rise_width=1800.0,
            fall_width=2500.0,
        )

        self.assertAlmostEqual(torque, 400.0)


    def test_rise_width_controls_torque_before_peak(self) -> None:
        torque = torque_curve(
            rpm=2700.0,
            peak_torque=400.0,
            base_torque=100.0,
            peak_rpm=4500.0,
            rise_width=1800.0,
            fall_width=2500.0,
        )

        expected = 100.0 + 300.0 * math.exp(-0.5)
        self.assertAlmostEqual(torque, expected)


    def test_fall_width_controls_torque_after_peak(self) -> None:
        torque = torque_curve(
            rpm=7000.0,
            peak_torque=400.0,
            base_torque=100.0,
            peak_rpm=4500.0,
            rise_width=1800.0,
            fall_width=2500.0,
        )

        distance = 7000.0 - 4500.0
        expected = 100.0 + 300.0 * math.exp(
            -(distance**2) / (2.0 * 2500.0**2)
        )
        self.assertAlmostEqual(torque, expected)




class TestEngine(unittest.TestCase):
    def test_engine_starts_at_idle_rpm(self) -> None:
        engine = make_engine()

        self.assertAlmostEqual(engine.rpm, 850.0)


    def test_angular_velocity_and_rpm_are_consistent(self) -> None:
        engine = make_engine(initial_rpm=3000.0)

        expected = 3000.0 * (2.0 * math.pi / 60.0)
        self.assertAlmostEqual(engine.angular_velocity, expected)
        self.assertAlmostEqual(engine.rpm, 3000.0)


    def test_full_throttle_produces_peak_torque(self) -> None:
        engine = make_engine(initial_rpm=4500.0)
        engine.set_throttle(1.0)

        self.assertAlmostEqual(
            engine.calculate_combustion_torque(),
            400.0,
        )


    def test_zero_throttle_produces_no_combustion_torque(self) -> None:
        engine = make_engine(initial_rpm=3000.0)
        engine.set_throttle(0.0)

        self.assertEqual(engine.calculate_combustion_torque(), 0.0)


    def test_throttle_is_clamped(self) -> None:
        engine = make_engine()

        engine.set_throttle(1.5)
        self.assertEqual(engine.throttle, 1.0)

        engine.set_throttle(-0.5)
        self.assertEqual(engine.throttle, 0.0)


    def test_redline_cuts_combustion_torque(self) -> None:
        engine = make_engine(initial_rpm=7000.0)
        engine.set_throttle(1.0)

        self.assertEqual(engine.calculate_combustion_torque(), 0.0)


    def test_friction_opposes_engine_rotation(self) -> None:
        engine = make_engine(initial_rpm=3000.0)

        self.assertLess(engine.calculate_friction_torque(), 0.0)


    def test_net_torque_does_not_change_engine_speed(self) -> None:
        engine = make_engine(initial_rpm=3000.0)
        engine.set_throttle(0.75)
        initial_rpm = engine.rpm

        engine.calculate_net_torque()

        self.assertAlmostEqual(engine.rpm, initial_rpm)


    def test_clutch_torque_is_included_in_net_torque(self) -> None:
        engine = make_engine(initial_rpm=3000.0)
        engine.set_throttle(1.0)

        without_clutch = engine.calculate_net_torque()
        with_clutch = engine.calculate_net_torque(clutch_torque=-50.0)

        self.assertAlmostEqual(with_clutch, without_clutch - 50.0)


    def test_invalid_inertia_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            make_engine(inertia=0.0)


    def test_invalid_torque_curve_width_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            make_engine(rise_width=0.0)


    def test_non_finite_throttle_is_rejected(self) -> None:
        engine = make_engine()

        with self.assertRaises(ValueError):
            engine.set_throttle(float("nan"))




if __name__ == "__main__":
    unittest.main()
