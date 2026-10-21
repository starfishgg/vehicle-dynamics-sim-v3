
"""Integration tests for a complete vehicle."""

import unittest

# from core.physics_constants import GRAVITY
from powertrain.clutch import Clutch
from powertrain.differential import Differential
from powertrain.drivetrain_solver import DrivetrainSolver
from powertrain.engine import Engine
from powertrain.gearbox import Gearbox
from tyres.tyre import Tyre
from tyres.tyre_parameters import TyreParameters
from vehicle.chassis import Chassis
from vehicle.vehicle import Vehicle
from vehicle.vehicle_dynamics import VehicleDynamics
from vehicle.wheel import Wheel

from configs.tyre_presets import create_road_tyre
from configs.engine_presets import create_sporty_petrol_engine




class VehicleDynamicsIntegrationTests(unittest.TestCase):
    """Test vehicle behaviour with real powertrain and tyre components."""

    def build_rear_wheel_drive_car(self):
        chassis = Chassis(
            dry_mass=1200.0,
            wheelbase=2.6,
            front_track=1.55,
            rear_track=1.53,
            centre_of_gravity_height=0.48,
            fuel_tank_capacity_litres=50.0,
            initial_fuel_litres=25.0,
        )

        front_left_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1800.0,
        )
        front_right_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1800.0,
        )
        rear_left_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1800.0,
        )
        rear_right_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1800.0,
        )

        front_left_tyre = create_road_tyre()
        front_right_tyre = create_road_tyre()
        rear_left_tyre = create_road_tyre()
        rear_right_tyre = create_road_tyre()

        engine = create_sporty_petrol_engine()
        engine.set_throttle(0.0)

        clutch = Clutch(max_torque=600.0, engagement=1.0)

        gearbox = Gearbox(
            forward_ratios=(3.5, 2.1, 1.4, 1.0),
            reverse_ratio=3.2,
            initial_gear=1,
        )

        differential = Differential()

        drivetrain = DrivetrainSolver(
            engine=engine,
            clutch=clutch,
            gearbox=gearbox,
            differential=differential,
            driven_wheels=(rear_left_wheel, rear_right_wheel),
        )

        vehicle = Vehicle(
            chassis=chassis,
            front_left_wheel=front_left_wheel,
            front_right_wheel=front_right_wheel,
            rear_left_wheel=rear_left_wheel,
            rear_right_wheel=rear_right_wheel,
            front_left_tyre=front_left_tyre,
            front_right_tyre=front_right_tyre,
            rear_left_tyre=rear_left_tyre,
            rear_right_tyre=rear_right_tyre,
            drivetrain=drivetrain,
        )

        dynamics = VehicleDynamics(vehicle)

        return vehicle, dynamics, engine


    def test_vehicle_accelerates_from_rest_under_throttle(self):
        vehicle, dynamics, engine = self.build_rear_wheel_drive_car()

        self.assertAlmostEqual(
            vehicle.state.longitudinal_velocity,
            0.0,
        )

        engine.set_throttle(1.0)

        dt = 1.0 / 120.0

        for _ in range(120):
            dynamics.step(dt)

        self.assertGreater(
            vehicle.state.longitudinal_velocity,
            0.0,
            "The vehicle should gain forward speed under throttle.",
        )

        self.assertGreater(
            vehicle.wheels[2].angular_velocity,
            0.0,
            "The rear driven wheel should rotate forwards.",
        )


    def test_braking_reduces_forward_speed(self):
        vehicle, dynamics, engine = self.build_rear_wheel_drive_car()
        engine.set_throttle(1.0)

        dt = 1.0 / 120.0

        for _ in range(120):
            dynamics.step(dt)

        speed_before_braking = vehicle.state.longitudinal_velocity
        self.assertGreater(speed_before_braking, 0.0)

        # Release throttle and apply brakes to all four wheels.
        engine.set_throttle(0.0)

        for wheel in vehicle.wheels:
            wheel.set_brake_input(1.0)

        for _ in range(120):
            dynamics.step(dt)

        self.assertLess(
            vehicle.state.longitudinal_velocity,
            speed_before_braking,
            "Braking should reduce forward speed.",
        )

        self.assertGreaterEqual(
            vehicle.state.longitudinal_velocity,
            0.0,
            "The vehicle should not reverse direction under braking.",
        )




if __name__ == "__main__":
    unittest.main()
