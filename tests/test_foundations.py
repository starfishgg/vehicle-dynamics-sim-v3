import unittest

from configs.simulation_config import SimulationConfig
from core.driver_input import DriverInput
from core.physics_state import PhysicsState




class SimulationConfigTests(unittest.TestCase):
    def test_default_timestep(self):
        config = SimulationConfig()

        self.assertAlmostEqual(
            config.fixed_timestep,
            1.0 / 120.0,
        )

    def test_invalid_timestep_is_rejected(self):
        with self.assertRaises(ValueError):
            SimulationConfig(fixed_timestep=0.0)

    def test_invalid_gravity_is_rejected(self):
        with self.assertRaises(ValueError):
            SimulationConfig(gravity=-1.0)




class DriverInputTests(unittest.TestCase):
    def test_inputs_are_clamped(self):
        controls = DriverInput(
            throttle=1.5,
            brake=-0.5,
            steering=2.0,
            requested_gear=3,
        )

        self.assertEqual(controls.throttle, 1.0)
        self.assertEqual(controls.brake, 0.0)
        self.assertEqual(controls.steering, 1.0)
        self.assertEqual(controls.requested_gear, 3)


    def test_no_gear_request_is_distinct_from_neutral(self):
        no_request = DriverInput()
        neutral_request = DriverInput(requested_gear=0)

        self.assertIsNone(no_request.requested_gear)
        self.assertEqual(neutral_request.requested_gear, 0)


    def test_invalid_gear_request_is_rejected(self):
        with self.assertRaises(TypeError):
            DriverInput(requested_gear=1.5)




class PhysicsStateTests(unittest.TestCase):
    def test_stationary_vehicle_has_zero_speed(self):
        state = PhysicsState()

        self.assertEqual(state.speed, 0.0)

    def test_speed_is_velocity_magnitude(self):
        state = PhysicsState()
        state.longitudinal_velocity = 3.0
        state.lateral_velocity = 4.0

        self.assertAlmostEqual(state.speed, 5.0)


if __name__ == "__main__":
    unittest.main()

