
"""Factories for assembling complete vehicles."""

from configs.engine_presets import create_sporty_petrol_engine
from configs.tyre_presets import create_road_tyre
from powertrain.clutch import Clutch
from powertrain.differential import Differential
from powertrain.drivetrain_solver import DrivetrainSolver
from powertrain.gearbox import Gearbox
from vehicle.chassis import Chassis
from vehicle.vehicle import Vehicle
from vehicle.vehicle_dynamics import VehicleDynamics
from vehicle.wheel import Wheel




def create_rear_wheel_drive_car() -> tuple[
    Vehicle, VehicleDynamics
]:
    """Build the standard test car used by the simulator."""
    chassis = Chassis(
        dry_mass=1200.0,
        wheelbase=2.6,
        front_track=1.55,
        rear_track=1.53,
        centre_of_gravity_height=0.48,
        fuel_tank_capacity_litres=50.0,
        initial_fuel_litres=25.0,
    )

    def create_wheel() -> Wheel:
        return Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1800.0,
        )

    front_left_wheel = create_wheel()
    front_right_wheel = create_wheel()
    rear_left_wheel = create_wheel()
    rear_right_wheel = create_wheel()

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

    return vehicle, VehicleDynamics(vehicle)

