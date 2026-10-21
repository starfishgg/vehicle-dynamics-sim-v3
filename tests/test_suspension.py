import unittest

from vehicle.suspension import Suspension




class TestSuspension(unittest.TestCase):
    def setUp(self) -> None:
        self.suspension = Suspension(
            spring_rate=30_000.0,
            damping_rate=3_000.0,
        )


    def test_force_from_spring_compression(self) -> None:
        force = self.suspension.calculate_force(
            compression=0.1,
            compression_velocity=0.0,
        )

        self.assertAlmostEqual(force, 3_000.0)


    def test_damper_increases_force_during_compression(self) -> None:
        force = self.suspension.calculate_force(
            compression=0.1,
            compression_velocity=0.5,
        )

        # 3000 N from the spring + 1500 N from the damper.
        self.assertAlmostEqual(force, 4_500.0)


    def test_damper_reduces_force_during_rebound(self) -> None:
        force = self.suspension.calculate_force(
            compression=0.1,
            compression_velocity=-0.5,
        )

        # 3000 N from the spring - 1500 N from the damper.
        self.assertAlmostEqual(force, 1_500.0)


    def test_force_is_zero_at_uncompressed_rest(self) -> None:
        force = self.suspension.calculate_force(0.0, 0.0)

        self.assertEqual(force, 0.0)


    def test_force_cannot_be_negative(self) -> None:
        force = self.suspension.calculate_force(
            compression=0.01,
            compression_velocity=-1.0,
        )

        self.assertEqual(force, 0.0)


    def test_zero_damping_is_valid(self) -> None:
        suspension = Suspension(
            spring_rate=30_000.0,
            damping_rate=0.0,
        )

        force = suspension.calculate_force(0.1, 1.0)

        self.assertAlmostEqual(force, 3_000.0)


    def test_invalid_spring_rate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Suspension(spring_rate=0.0, damping_rate=3_000.0)


    def test_negative_damping_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Suspension(spring_rate=30_000.0, damping_rate=-1.0)


    def test_non_finite_compression_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.suspension.calculate_force(float("nan"), 0.0)


    def test_non_finite_velocity_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.suspension.calculate_force(0.1, float("inf"))


if __name__ == "__main__":
    unittest.main()
