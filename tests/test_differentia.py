import unittest

from powertrain.differential import Differential




class TestDifferential(unittest.TestCase):
    def setUp(self) -> None:
        self.differential = Differential(efficiency=0.97)


    def test_equal_torque_distribution(self) -> None:
        left_torque, right_torque = (
            self.differential.calculate_output_torques(200.0)
        )

        self.assertAlmostEqual(left_torque, 97.0)
        self.assertAlmostEqual(right_torque, 97.0)


    def test_torque_distribution_accounts_for_efficiency(self) -> None:
        differential = Differential(efficiency=1.0)

        left_torque, right_torque = (
            differential.calculate_output_torques(200.0)
        )

        self.assertAlmostEqual(left_torque, 100.0)
        self.assertAlmostEqual(right_torque, 100.0)


    def test_negative_input_torque_is_distributed_equally(self) -> None:
        left_torque, right_torque = (
            self.differential.calculate_output_torques(-200.0)
        )

        self.assertAlmostEqual(left_torque, -97.0)
        self.assertAlmostEqual(right_torque, -97.0)


    def test_carrier_speed_is_average_of_wheel_speeds(self) -> None:
        carrier_speed = (
            self.differential.calculate_input_angular_velocity(
                left_wheel_angular_velocity=80.0,
                right_wheel_angular_velocity=120.0,
            )
        )

        self.assertAlmostEqual(carrier_speed, 100.0)


    def test_equal_wheel_speeds_match_carrier_speed(self) -> None:
        carrier_speed = (
            self.differential.calculate_input_angular_velocity(
                left_wheel_angular_velocity=100.0,
                right_wheel_angular_velocity=100.0,
            )
        )

        self.assertAlmostEqual(carrier_speed, 100.0)


    def test_right_wheel_speed_can_be_calculated(self) -> None:
        right_speed = (
            self.differential.calculate_right_wheel_angular_velocity(
                input_angular_velocity=100.0,
                left_wheel_angular_velocity=80.0,
            )
        )

        self.assertAlmostEqual(right_speed, 120.0)


    def test_left_wheel_speed_can_be_calculated(self) -> None:
        left_speed = (
            self.differential.calculate_left_wheel_angular_velocity(
                input_angular_velocity=100.0,
                right_wheel_angular_velocity=120.0,
            )
        )

        self.assertAlmostEqual(left_speed, 80.0)


    def test_wheel_speeds_satisfy_differential_relationship(self) -> None:
        left_speed = 60.0
        right_speed = 140.0

        carrier_speed = (
            self.differential.calculate_input_angular_velocity(
                left_speed,
                right_speed,
            )
        )

        self.assertAlmostEqual(carrier_speed, 100.0)


    def test_negative_wheel_speeds_are_supported(self) -> None:
        carrier_speed = (
            self.differential.calculate_input_angular_velocity(
                left_wheel_angular_velocity=-80.0,
                right_wheel_angular_velocity=-120.0,
            )
        )

        self.assertAlmostEqual(carrier_speed, -100.0)


    def test_non_finite_input_torque_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.differential.calculate_output_torques(float("nan"))


    def test_non_finite_wheel_speed_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.differential.calculate_input_angular_velocity(
                float("inf"),
                100.0,
            )


    def test_invalid_efficiency_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Differential(efficiency=0.0)


    def test_output_torques_split_input_torque_with_efficiency(self):
        differential = Differential(efficiency=0.97)

        left_torque, right_torque = differential.calculate_output_torques(200.0)

        self.assertAlmostEqual(left_torque, 97.0)
        self.assertAlmostEqual(right_torque, 97.0)
        self.assertAlmostEqual(left_torque + right_torque, 200.0 * 0.97)




if __name__ == "__main__":
    unittest.main()
