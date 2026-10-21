import unittest

from powertrain.gearbox import Gearbox




class TestGearbox(unittest.TestCase):
    def setUp(self) -> None:
        self.gearbox = Gearbox(
            forward_ratios=(3.5, 2.1, 1.4, 1.0, 0.8),
            reverse_ratio=3.2,
            efficiency=0.97,
        )

    def test_starts_in_neutral_by_default(self) -> None:
        self.assertEqual(self.gearbox.selected_gear, 0)
        self.assertTrue(self.gearbox.is_neutral)
        self.assertIsNone(self.gearbox.gear_ratio)


    def test_select_first_gear(self) -> None:
        self.gearbox.select_gear(1)

        self.assertEqual(self.gearbox.selected_gear, 1)
        self.assertEqual(self.gearbox.gear_ratio, 3.5)


    def test_select_reverse(self) -> None:
        self.gearbox.select_gear(-1)

        self.assertEqual(self.gearbox.gear_ratio, -3.2)


    def test_select_neutral(self) -> None:
        self.gearbox.select_gear(2)
        self.gearbox.select_gear(0)

        self.assertTrue(self.gearbox.is_neutral)
        self.assertIsNone(self.gearbox.gear_ratio)


    def test_invalid_gear_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.gearbox.select_gear(6)

        with self.assertRaises(ValueError):
            self.gearbox.select_gear(-2)


    def test_non_integer_gear_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.gearbox.select_gear(1.5)

        with self.assertRaises(ValueError):
            self.gearbox.select_gear(True)


    def test_output_speed_uses_selected_ratio(self) -> None:
        self.gearbox.select_gear(1)

        output_speed = self.gearbox.calculate_output_angular_velocity(
            350.0
        )

        self.assertAlmostEqual(output_speed, 100.0)


    def test_reverse_reverses_output_rotation(self) -> None:
        self.gearbox.select_gear(-1)

        output_speed = self.gearbox.calculate_output_angular_velocity(
            320.0
        )

        self.assertAlmostEqual(output_speed, -100.0)


    def test_neutral_does_not_constrain_output_speed(self) -> None:
        self.assertIsNone(
            self.gearbox.calculate_output_angular_velocity(200.0)
        )


    def test_output_torque_includes_efficiency(self) -> None:
        self.gearbox.select_gear(1)

        output_torque = self.gearbox.calculate_output_torque(100.0)

        self.assertAlmostEqual(output_torque, 3.5 * 100.0 * 0.97)


    def test_neutral_transmits_no_torque(self) -> None:
        self.assertEqual(
            self.gearbox.calculate_output_torque(400.0),
            0.0,
        )


    def test_reverse_reverses_output_torque(self) -> None:
        self.gearbox.select_gear(-1)

        output_torque = self.gearbox.calculate_output_torque(100.0)

        self.assertAlmostEqual(output_torque, -3.2 * 100.0 * 0.97)


    def test_input_speed_can_be_calculated_from_output_speed(self) -> None:
        self.gearbox.select_gear(2)

        input_speed = self.gearbox.calculate_input_angular_velocity(
            100.0
        )

        self.assertAlmostEqual(input_speed, 210.0)


    def test_neutral_has_no_fixed_input_speed_relationship(self) -> None:
        self.assertIsNone(
            self.gearbox.calculate_input_angular_velocity(100.0)
        )


    def test_forward_gear_count(self) -> None:
        self.assertEqual(self.gearbox.gear_count, 5)


    def test_invalid_efficiency_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Gearbox(
                forward_ratios=(3.5, 2.1),
                reverse_ratio=3.2,
                efficiency=0.0,
            )


    def test_invalid_forward_ratio_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Gearbox(
                forward_ratios=(3.5, 0.0),
                reverse_ratio=3.2,
            )


    def test_output_torque_accounts_for_gear_ratio_and_efficiency(self):
        gearbox = Gearbox(
            forward_ratios=(3.5, 2.0),
            reverse_ratio=3.0,
            efficiency=0.97,
            initial_gear=1,
        )

        input_torque = 100.0
        output_torque = gearbox.calculate_output_torque(input_torque)

        self.assertAlmostEqual(
            output_torque,
            input_torque * 3.5 * 0.97,
        )




if __name__ == "__main__":
    unittest.main()
