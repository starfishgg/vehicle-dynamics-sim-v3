import math
import unittest

from core.magic_formulas import magic_formula
from tyres.tyre import Tyre
from tyres.tyre_parameters import TyreParameters




class TestMagicFormula(unittest.TestCase):
    def test_zero_slip_produces_zero_response(self) -> None:
        self.assertAlmostEqual(
            magic_formula(0.0, 10.0, 1.9, 1.0, 0.97),
            0.0,
        )


    def test_positive_and_negative_slip_have_opposite_signs(self) -> None:
        positive = magic_formula(0.1, 10.0, 1.9, 1.0, 0.97)
        negative = magic_formula(-0.1, 10.0, 1.9, 1.0, 0.97)

        self.assertGreater(positive, 0.0)
        self.assertLess(negative, 0.0)
        self.assertAlmostEqual(positive, -negative)


    def test_invalid_stiffness_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            magic_formula(0.1, 0.0, 1.9, 1.0, 0.97)


    def test_non_finite_input_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            magic_formula(float("nan"), 10.0, 1.9, 1.0, 0.97)




class TestTyreParameters(unittest.TestCase):
    def test_default_parameters_are_available(self) -> None:
        parameters = TyreParameters()

        self.assertEqual(parameters.friction_coefficient, 1.0)
        self.assertEqual(parameters.longitudinal_stiffness, 10.0)


    def test_non_positive_friction_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TyreParameters(friction_coefficient=0.0)




class TestTyre(unittest.TestCase):
    def setUp(self) -> None:
        self.tyre = Tyre(TyreParameters())


    def test_zero_slip_produces_no_longitudinal_force(self) -> None:
        force = self.tyre.calculate_longitudinal_force(0.0, 4000.0)

        self.assertAlmostEqual(force, 0.0)


    def test_positive_longitudinal_slip_produces_forward_force(self) -> None:
        force = self.tyre.calculate_longitudinal_force(0.1, 4000.0)

        self.assertGreater(force, 0.0)


    def test_negative_longitudinal_slip_produces_backward_force(self) -> None:
        force = self.tyre.calculate_longitudinal_force(-0.1, 4000.0)

        self.assertLess(force, 0.0)


    def test_force_scales_with_normal_load(self) -> None:
        force_at_2000_n = self.tyre.calculate_longitudinal_force(0.1, 2000.0)
        force_at_4000_n = self.tyre.calculate_longitudinal_force(0.1, 4000.0)

        self.assertAlmostEqual(force_at_4000_n, 2.0 * force_at_2000_n)


    def test_force_scales_with_friction_coefficient(self) -> None:
        low_grip = Tyre(TyreParameters(friction_coefficient=0.5))
        standard_grip = Tyre(TyreParameters(friction_coefficient=1.0))

        low_force = low_grip.calculate_longitudinal_force(0.1, 4000.0)
        standard_force = standard_grip.calculate_longitudinal_force(0.1, 4000.0)

        self.assertAlmostEqual(low_force, 0.5 * standard_force)


    def test_zero_normal_load_produces_zero_force(self) -> None:
        force = self.tyre.calculate_lateral_force(0.1, 0.0)

        self.assertAlmostEqual(force, 0.0)


    def test_positive_slip_angle_produces_leftward_force(self) -> None:
        force = self.tyre.calculate_lateral_force(0.1, 4000.0)

        self.assertGreater(force, 0.0)


    def test_negative_slip_angle_produces_rightward_force(self) -> None:
        force = self.tyre.calculate_lateral_force(-0.1, 4000.0)

        self.assertLess(force, 0.0)


    def test_reaction_torque_opposes_longitudinal_force(self) -> None:
        torque = Tyre.calculate_reaction_torque(1000.0, 0.32)

        self.assertAlmostEqual(torque, -320.0)


    def test_reverse_force_produces_opposite_reaction_torque(self) -> None:
        torque = Tyre.calculate_reaction_torque(-1000.0, 0.32)

        self.assertAlmostEqual(torque, 320.0)


    def test_invalid_normal_load_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.tyre.calculate_longitudinal_force(0.1, -100.0)


    def test_invalid_wheel_radius_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Tyre.calculate_reaction_torque(100.0, 0.0)




if __name__ == "__main__":
    unittest.main()
