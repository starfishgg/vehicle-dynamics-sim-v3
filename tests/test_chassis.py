
import unittest

from vehicle.chassis import Chassis




class DynamicAxleLoadTests(unittest.TestCase):
    def setUp(self):
        self.chassis = Chassis(
            dry_mass=1200.0,
            wheelbase=2.6,
            front_track=1.55,
            rear_track=1.53,
            centre_of_gravity_height=0.48,
            fuel_tank_capacity_litres=50.0,
            initial_fuel_litres=25.0,
        )


    def test_zero_acceleration_preserves_static_loads(self):
        static_loads = self.chassis.calculate_static_axle_loads()
        dynamic_loads = self.chassis.calculate_dynamic_axle_loads(0.0)

        self.assertAlmostEqual(dynamic_loads[0], static_loads[0])
        self.assertAlmostEqual(dynamic_loads[1], static_loads[1])


    def test_acceleration_transfers_load_to_rear(self):
        static_front, static_rear = (
            self.chassis.calculate_static_axle_loads()
        )
        dynamic_front, dynamic_rear = (
            self.chassis.calculate_dynamic_axle_loads(3.0)
        )

        self.assertLess(dynamic_front, static_front)
        self.assertGreater(dynamic_rear, static_rear)


    def test_braking_transfers_load_to_front(self):
        static_front, static_rear = (
            self.chassis.calculate_static_axle_loads()
        )
        dynamic_front, dynamic_rear = (
            self.chassis.calculate_dynamic_axle_loads(-5.0)
        )

        self.assertGreater(dynamic_front, static_front)
        self.assertLess(dynamic_rear, static_rear)


    def test_total_axle_load_is_conserved(self):
        static_total = sum(self.chassis.calculate_static_axle_loads())

        for acceleration in (-5.0, 0.0, 3.0):
            with self.subTest(acceleration=acceleration):
                front, rear = (
                    self.chassis.calculate_dynamic_axle_loads(
                        acceleration
                    )
                )
                self.assertAlmostEqual(front + rear, static_total)


    def test_extreme_acceleration_does_not_create_negative_loads(self):
        front, rear = (
            self.chassis.calculate_dynamic_axle_loads(1000.0)
        )

        self.assertGreaterEqual(front, 0.0)
        self.assertGreaterEqual(rear, 0.0)
        self.assertAlmostEqual(
            front + rear,
            sum(self.chassis.calculate_static_axle_loads()),
        )




if __name__ == "__main__":
    unittest.main()
