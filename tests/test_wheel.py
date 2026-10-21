import math
import unittest

from vehicle.wheel import Wheel




class TestWheel(unittest.TestCase):
    def setUp(self) -> None:
        self.wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1000.0,
        )


    def test_wheel_starts_stationary(self) -> None:
        self.assertEqual(self.wheel.angular_velocity, 0.0)
        self.assertEqual(self.wheel.rotational_speed_rpm, 0.0)
        self.assertEqual(self.wheel.linear_speed, 0.0)


    def test_rotational_speed_rpm_conversion(self) -> None:
        self.wheel.angular_velocity = 100.0

        expected_rpm = 100.0 * 60.0 / (2.0 * math.pi)
        self.assertAlmostEqual(self.wheel.rotational_speed_rpm, expected_rpm)


    def test_linear_speed_uses_wheel_radius(self) -> None:
        self.wheel.angular_velocity = 50.0

        self.assertAlmostEqual(self.wheel.linear_speed, 16.0)


    def test_reverse_rotation_has_negative_linear_speed(self) -> None:
        self.wheel.angular_velocity = -50.0

        self.assertAlmostEqual(self.wheel.linear_speed, -16.0)
        self.assertLess(self.wheel.rotational_speed_rpm, 0.0)


    def test_drive_torque_is_applied(self) -> None:
        self.wheel.set_drive_torque(120.0)

        self.assertAlmostEqual(self.wheel.calculate_net_torque(), 120.0)
        self.assertAlmostEqual(
            self.wheel.calculate_angular_acceleration(),
            100.0,
        )


    def test_tyre_reaction_torque_is_included(self) -> None:
        self.wheel.set_drive_torque(120.0)
        self.wheel.set_tyre_reaction_torque(-30.0)

        self.assertAlmostEqual(self.wheel.calculate_net_torque(), 90.0)


    def test_brake_input_is_clamped(self) -> None:
        self.wheel.set_brake_input(1.5)
        self.assertEqual(self.wheel.brake_input, 1.0)

        self.wheel.set_brake_input(-0.5)
        self.assertEqual(self.wheel.brake_input, 0.0)


    def test_brake_opposes_forward_rotation(self) -> None:
        self.wheel.angular_velocity = 10.0
        self.wheel.set_brake_input(0.5)

        self.assertAlmostEqual(self.wheel.calculate_brake_torque(), -500.0)


    def test_brake_opposes_reverse_rotation(self) -> None:
        self.wheel.angular_velocity = -10.0
        self.wheel.set_brake_input(0.5)

        self.assertAlmostEqual(self.wheel.calculate_brake_torque(), 500.0)


    def test_brake_holds_stationary_wheel_against_small_torque(self) -> None:
        self.wheel.set_drive_torque(100.0)
        self.wheel.set_brake_input(0.5)

        self.assertAlmostEqual(self.wheel.calculate_brake_torque(), -100.0)
        self.assertAlmostEqual(self.wheel.calculate_net_torque(), 0.0)


    def test_brake_cannot_hold_against_excessive_torque(self) -> None:
        self.wheel.set_drive_torque(600.0)
        self.wheel.set_brake_input(0.5)

        self.assertAlmostEqual(self.wheel.calculate_brake_torque(), -500.0)
        self.assertAlmostEqual(self.wheel.calculate_net_torque(), 100.0)


    def test_integrate_updates_angular_velocity(self) -> None:
        self.wheel.set_drive_torque(120.0)
        self.wheel.integrate_angular_velocity(0.1)

        self.assertAlmostEqual(self.wheel.angular_velocity, 10.0)


    def test_braking_slows_forward_rotation(self) -> None:
        self.wheel.angular_velocity = 10.0
        self.wheel.set_brake_input(0.5)
        self.wheel.integrate_angular_velocity(0.01)

        self.assertAlmostEqual(self.wheel.angular_velocity, 5.833333333333333)


    def test_braking_does_not_numerically_reverse_wheel(self) -> None:
        self.wheel.angular_velocity = 1.0
        self.wheel.set_brake_input(1.0)
        self.wheel.integrate_angular_velocity(0.1)

        self.assertEqual(self.wheel.angular_velocity, 0.0)


    def test_invalid_wheel_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Wheel(radius=0.0, inertia=1.2, max_brake_torque=1000.0)

        with self.assertRaises(ValueError):
            Wheel(radius=0.32, inertia=0.0, max_brake_torque=1000.0)


    def test_invalid_timestep_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.wheel.integrate_angular_velocity(0.0)


    def test_non_finite_drive_torque_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.wheel.set_drive_torque(float("nan"))




if __name__ == "__main__":
    unittest.main()
