import unittest

from powertrain.clutch import Clutch




class TestClutch(unittest.TestCase):
    def setUp(self) -> None:
        self.clutch = Clutch(
            max_torque=400.0,
            engagement=1.0,
            lock_slip_threshold=0.5,
        )


    def test_locked_clutch_supplies_required_torque(self) -> None:
        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=100.0,
            gearbox_angular_velocity=100.1,
            required_engine_torque=-120.0,
        )

        self.assertAlmostEqual(torque, -120.0)


    def test_required_torque_is_limited_by_capacity(self) -> None:
        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=100.0,
            gearbox_angular_velocity=100.1,
            required_engine_torque=-450.0,
        )

        self.assertAlmostEqual(torque, -400.0)


    def test_slipping_clutch_opposes_relative_rotation(self) -> None:
        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=120.0,
            gearbox_angular_velocity=100.0,
            required_engine_torque=50.0,
        )

        self.assertAlmostEqual(torque, -400.0)


    def test_clutch_transfers_torque_when_engine_is_slower(self) -> None:
        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=80.0,
            gearbox_angular_velocity=100.0,
            required_engine_torque=-50.0,
        )

        self.assertAlmostEqual(torque, 400.0)


    def test_disengaged_clutch_transmits_no_torque(self) -> None:
        self.clutch.engagement = 0.0

        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=120.0,
            gearbox_angular_velocity=100.0,
            required_engine_torque=-100.0,
        )

        self.assertAlmostEqual(torque, 0.0)


    def test_clutch_cannot_lock_when_torque_exceeds_capacity(self) -> None:
        locked = self.clutch.is_locked(
            engine_angular_velocity=100.0,
            gearbox_angular_velocity=100.1,
            required_engine_torque=450.0,
        )

        self.assertFalse(locked)


    def test_torque_capacity_scales_with_engagement(self):
        self.clutch.engagement = 0.25
        self.assertAlmostEqual(self.clutch.torque_capacity, 100.0)

        self.clutch.engagement = 0.50
        self.assertAlmostEqual(self.clutch.torque_capacity, 200.0)

        self.clutch.engagement = 1.00
        self.assertAlmostEqual(self.clutch.torque_capacity, 400.0)


    def test_engagement_is_clamped_to_valid_range(self):
        self.clutch.engagement = 1.5
        self.assertAlmostEqual(self.clutch.engagement, 1.0)

        self.clutch.engagement = -0.5
        self.assertAlmostEqual(self.clutch.engagement, 0.0)


    def test_partial_engagement_limits_transmitted_torque(self):
        self.clutch.engagement = 0.25

        torque = self.clutch.calculate_transmitted_torque(
            engine_angular_velocity=120.0,
            gearbox_angular_velocity=100.0,
            required_engine_torque=50.0,
        )

        self.assertAlmostEqual(torque, -100.0)





if __name__ == "__main__":
    unittest.main()

