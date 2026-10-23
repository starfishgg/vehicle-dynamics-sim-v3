"""Longitudinal vehicle dynamics and road-load calculations"""

import math

from core.physics_constants import AIR_DENSITY, DRY_ASPHALT, GRAVITY
from core.driver_input import DriverInput
from vehicle.vehicle import Vehicle
from powertrain.clutch_controller import ClutchController




# Below this speed, use a low-speed reference when calculating tyre slip.
# This avoids division by zero and excessive slip ratios near standstill.
MINIMUM_SLIP_SPEED: float = 0.5

# Treat the vehicle as stationary for rolling-resistance purposes below
# this speed, avoiding a discotinuity when starting from rest.
MINIMUM_ROLLING_SPEED: float = 0.01




class VehicleDynamics:
    """Calculate longitudinal tyre forces and vehicle body motion."""

    def __init__(
            self,
            vehicle: Vehicle,
            *,
            surface_grip: float = DRY_ASPHALT,
    ) -> None:
        if not math.isfinite(surface_grip) or surface_grip < 0.0:
            raise ValueError("Surface grip must be finite and non-negative.")

        self.vehicle = vehicle
        self.surface_grip = surface_grip
        self.clutch_controller = ClutchController(
            vehicle.drivetrain.clutch
        )

        # Keep the most recent force calculations available for debugging.
        self.longitudinal_tyre_forces: tuple[float, ...] = (
            0.0, 0.0, 0.0, 0.0
        )

        self.drag_force: float = 0.0
        self.rolling_resistance_force: float = 0.0
        self.net_longitudinal_force: float = 0.0


    @staticmethod
    def calculate_slip_ratio(
        wheel_linear_speed: float,
        vehicle_speed: float,
    ) -> float:
        """Calculate longitudinal slip with a low-speed safeguard."""
        speed_difference = wheel_linear_speed - vehicle_speed

        reference_speed = max(
            abs(wheel_linear_speed),
            abs(vehicle_speed),
            MINIMUM_SLIP_SPEED,
        )

        return speed_difference / reference_speed


    def _calculate_tyre_forces(self) -> tuple[float, ...]:
        """Calculate each tyre's longitudinal force from current wheel slip."""
        state = self.vehicle.state

        front_axle_load, rear_axle_load = (
            self.vehicle.chassis.calculate_dynamic_axle_loads(
                self.vehicle.state.longitudinal_acceleration,
                GRAVITY,
            )
        )

        axle_loads = (
            front_axle_load / 2.0,
            front_axle_load / 2.0,
            rear_axle_load / 2.0,
            rear_axle_load / 2.0,
        )

        forces: list[float] = []

        for wheel, tyre, normal_load in zip(
            self.vehicle.wheels,
            self.vehicle.tyres,
            axle_loads,
        ):
            slip_ratio = self.calculate_slip_ratio(
                wheel.linear_speed,
                state.longitudinal_velocity,
            )

            # The surface multiplier scales the tyre's available grip.
            tyre_force = (
                tyre.calculate_longitudinal_force(
                    slip_ratio,
                    normal_load,
                )
                * self.surface_grip
            )

            reaction_torque = tyre.calculate_reaction_torque(
                tyre_force,
                wheel.radius,
            )

            wheel.set_tyre_reaction_torque(reaction_torque)
            forces.append(tyre_force)

        return tuple(forces)


    def _calculate_drag_force(self, speed: float) -> float:
        """Calculate aerodynamic drag, opposing the direction of travel."""
        chassis = self.vehicle.chassis

        return (
            -0.5
            * AIR_DENSITY
            * chassis.drag_coefficient
            * chassis.frontal_area
            * speed
            * abs(speed)
        )


    def _calculate_rolling_resistance(self, speed: float) -> float:
        """Calculate rolling resistance opposing vehicle motion."""
        if abs(speed) < MINIMUM_ROLLING_SPEED:
            return 0.0

        chassis = self.vehicle.chassis

        resistance_magnitude = (
            chassis.rolling_resistance_coefficient
            * chassis.total_mass
            * GRAVITY
        )

        return -math.copysign(resistance_magnitude, speed)


    def step(
            self, 
            dt: float,
            driver_input: DriverInput | None = None,
    ) -> None:
        """Advance wheel and longitudinal body dynamics by one time step."""
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("Time step must be finite and positive.")

        if driver_input is None:
            driver_input = DriverInput()
        
        vehicle = self.vehicle
        state = vehicle.state

        # Apply player controls through the existing component interfaces.

        # Apply throttle
        vehicle.drivetrain.engine.set_throttle(driver_input.throttle)

        # Apply brake
        for wheel in vehicle.wheels:
            wheel.set_brake_input(driver_input.brake)

        # Apply clutch-pedal input to the existing drivetrain clutch.
        self.clutch_controller.update(driver_input.clutch_pedal)


        #print(
        #    f"DEBUG pedal={driver_input.clutch_pedal:.1f} "
        #    f"engagement={vehicle.drivetrain.clutch.engagement:.1f}"
        #)


        # Process a requested gear change, if one was supplied.
        if driver_input.requested_gear is not None:
            vehicle.drivetrain.gearbox.select_gear(driver_input.requested_gear)

        # 1. Calculate tyre forces and feed their reaction torques into
        #    the wheel and drivetrain models.
        tyre_forces = self._calculate_tyre_forces()
        self.longitudinal_tyre_forces = tyre_forces

        # 2. The drivetrain sovler owns driven-wheel integration.
        #    Integrate only the wheels that it does not control here.
        drive_wheel_ids = {
            id(wheel) for wheel in vehicle.drivetrain.driven_wheels
        }

        for wheel in vehicle.wheels:
            if id(wheel) not in drive_wheel_ids:
                wheel.integrate_angular_velocity(dt)

        # 3. Advance the engine, clutch, gearbox, differential and
        #    driven wheel using the reaction torques calculated above.
        vehicle.drivetrain.step(dt)

        # 4. Calculate the net longitudinal force on the vehicle body.
        speed = state.longitudinal_velocity

        self.drag_force = self._calculate_drag_force(speed)
        self.rolling_resistance_force = (
            self._calculate_rolling_resistance(speed)
        )

        total_tyre_force = sum(tyre_forces)

        self.net_longitudinal_force = (
            total_tyre_force
            + self.drag_force
            + self.rolling_resistance_force
        )

        # Fuel consumption affects chassis.total_mass, so acceleration
        # automatically reflects the vehicle's current mass.
        acceleration = (
            self.net_longitudinal_force / vehicle.chassis.total_mass
        )

        state.longitudinal_acceleration = acceleration

        # Semi-implicit Euler integration: update velocity first, then
        # use the updated velocity to advance world position.
        state.longitudinal_velocity += acceleration * dt

        cos_heading = math.cos(state.heading)
        sin_heading = math.sin(state.heading)

        world_velocity_x = (
            state.longitudinal_velocity * cos_heading
            - state.lateral_velocity * sin_heading
        )

        world_velocity_y = (
            state.longitudinal_velocity * sin_heading
            + state.lateral_velocity * cos_heading
        )

        state.position_x += world_velocity_x * dt
        state.position_y += world_velocity_y * dt
