
import math

from powertrain.clutch import Clutch
from powertrain.differential import Differential
from powertrain.engine import Engine
from powertrain.gearbox import Gearbox
from vehicle.wheel import Wheel


class DrivetrainSolver:
    """
    Coordinate engine and driven-wheel rotational dynamics.

    The solver owns integration of engine and driven-wheel angular velocity.
    Tyre reaction torque is supplied by the vehicle/tyre calculations.

    A clutch can operate in two modes:
    - Locked: engine and gearbox input speeds are constrained to match.
    - Slipping: clutch torque opposes relative rotation, and engine and
      wheels are integrated independently.

    Speed synchronisation is calculated as a candidate outcome first. The
    component states are changed only after the solver has selected a mode.
    """

    def __init__(
        self,
        *,
        engine: Engine,
        clutch: Clutch,
        gearbox: Gearbox,
        differential: Differential,
        driven_wheels: tuple[Wheel, Wheel],
        final_drive_ratio: float = 3.9,
        final_drive_efficiency: float = 0.97,
    ) -> None:
        if len(driven_wheels) != 2:
            raise ValueError("Exactly two driven wheels are required.")

        if (
            not math.isfinite(final_drive_ratio)
            or final_drive_ratio <= 0.0
        ):
            raise ValueError("final_drive_ratio must be finite and positive.")

        if (
            not math.isfinite(final_drive_efficiency)
            or not 0.0 < final_drive_efficiency <= 1.0
        ):
            raise ValueError(
                "final_drive_efficiency must be in the range (0.0, 1.0]."
            )

        self.engine = engine
        self.clutch = clutch
        self.gearbox = gearbox
        self.differential = differential
        self.driven_wheels = driven_wheels

        self.final_drive_ratio = final_drive_ratio
        self.final_drive_efficiency = final_drive_efficiency

        # These values describe the most recently completed solver step.
        self.clutch_torque = 0.0
        self.gearbox_input_angular_velocity = engine.angular_velocity


    def calculate_gearbox_input_speed(self) -> float:
        """Calculate gearbox input speed from the driven-wheel speeds."""
        if self.gearbox.gear_ratio is None:
            # In neutral, wheel rotation does not determine engine speed.
            return self.engine.angular_velocity

        left_wheel, right_wheel = self.driven_wheels

        carrier_speed = self.differential.calculate_input_angular_velocity(
            left_wheel.angular_velocity,
            right_wheel.angular_velocity,
        )

        gearbox_output_speed = (
            carrier_speed * self.final_drive_ratio
        )

        return self.gearbox.calculate_input_angular_velocity(gearbox_output_speed)


    def _calculate_gearbox_speed_for_wheels(
        self,
        wheel_speeds: tuple[float, float],
    ) -> float:
        """
        Calculate gearbox input speed for candidate wheel speeds.

        This lets the solver evaluate a possible clutch lock without first
        changing the actual wheel or engine state.
        """
        gear_ratio = self.gearbox.gear_ratio

        if gear_ratio is None:
            return self.engine.angular_velocity

        carrier_speed = self.differential.calculate_input_angular_velocity(
            wheel_speeds[0],
            wheel_speeds[1],
        )

        gearbox_output_speed = (
            carrier_speed * self.final_drive_ratio
        )

        return self.gearbox.calculate_input_angular_velocity(gearbox_output_speed)


    @staticmethod
    def _calculate_external_wheel_torque(
        wheel: Wheel,
        drive_torque: float,
        angular_velocity: float,
    ) -> float:
        """
        Calculate tyre reaction and brake torque without changing the wheel.

        At non-zero speed, the brake opposes wheel rotation. At zero speed,
        the brake opposes the combined drive and tyre torque, up to its
        maximum capacity. This matches Wheel.calculate_brake_torque().
        """
        tyre_torque = wheel.tyre_reaction_torque
        brake_capacity = wheel.brake_torque_capacity

        if brake_capacity == 0.0:
            brake_torque = 0.0
        elif angular_velocity > 0.0:
            brake_torque = -brake_capacity
        elif angular_velocity < 0.0:
            brake_torque = brake_capacity
        else:
            # A stationary wheel can remain stationary while the brakes
            # cancel an applied torque, provided their capacity is enough.
            applied_torque = drive_torque + tyre_torque

            if applied_torque == 0.0:
                brake_torque = 0.0
            else:
                brake_torque = -math.copysign(
                    min(abs(applied_torque), brake_capacity),
                    applied_torque,
                )

        return tyre_torque + brake_torque


    def _calculate_locked_clutch_torque(
        self,
        engine_free_torque: float,
        gear_ratio: float,
        wheel_speeds: tuple[float, float],
    ) -> float:
        """
        Calculate the clutch torque needed to maintain locked motion.

        The engine acceleration must equal the gearbox input acceleration.
        The equation includes engine inertia, both wheel inertias, gear
        ratios and transmission efficiencies.

        Brake torque at zero wheel speed depends on the applied drive torque.
        A short fixed-point iteration accounts for that dependency without
        temporarily changing either wheel's drive torque.
        """
        left_wheel, right_wheel = self.driven_wheels

        total_efficiency = (
            self.gearbox.efficiency
            * self.final_drive_efficiency
            * self.differential.efficiency
        )

        overall_ratio = gear_ratio * self.final_drive_ratio

        inverse_wheel_inertia = (
            1.0 / left_wheel.inertia
            + 1.0 / right_wheel.inertia
        )

        # Clutch torque is applied to the engine; the gearbox receives its
        # negative. The signed gear ratio determines the direction of torque.
        denominator = (
            1.0 / self.engine.inertia
            + overall_ratio**2
            * total_efficiency
            * inverse_wheel_inertia
            / 4.0
        )

        clutch_torque = 0.0

        for _ in range(12):
            gearbox_output_torque = (
                -clutch_torque
                * gear_ratio
                * self.gearbox.efficiency
                * self.final_drive_ratio
                * self.final_drive_efficiency
            )

            left_drive_torque, right_drive_torque = (
                self.differential.calculate_output_torques(
                    gearbox_output_torque
                )
            )

            # Use candidate wheel speeds to choose the correct brake
            # direction, including when synchronisation changes wheel speed.
            left_external_torque = self._calculate_external_wheel_torque(
                left_wheel,
                left_drive_torque,
                wheel_speeds[0],
            )
            right_external_torque = self._calculate_external_wheel_torque(
                right_wheel,
                right_drive_torque,
                wheel_speeds[1],
            )

            external_acceleration_term = (
                left_external_torque / left_wheel.inertia
                + right_external_torque / right_wheel.inertia
            )

            # The differential carrier speed is the mean of the wheel
            # speeds, so its acceleration is the mean of wheel accelerations.
            numerator = (
                overall_ratio * external_acceleration_term / 2.0
                - engine_free_torque / self.engine.inertia
            )

            next_clutch_torque = numerator / denominator

            if math.isclose(
                next_clutch_torque,
                clutch_torque,
                rel_tol=1e-9,
                abs_tol=1e-9,
            ):
                return next_clutch_torque

            clutch_torque = next_clutch_torque

        # Return the last estimate if stationary-wheel brake behaviour did
        # not converge exactly within the iteration limit.
        return clutch_torque


    def _calculate_locked_speed_candidate(
        self,
    ) -> tuple[float, tuple[float, float]] | None:
        """
        Calculate an inertia-weighted synchronisation without mutating state.

        The correction conserves angular momentum for the idealised
        engine-and-wheel inertias used here. Relative rotational energy is
        dissipated during synchronisation, as expected when a slipping
        clutch engages.

        Return None if the correction would require negative engine speed.
        """
        gear_ratio = self.gearbox.gear_ratio

        if gear_ratio is None:
            return None

        overall_ratio = gear_ratio * self.final_drive_ratio

        left_wheel, right_wheel = self.driven_wheels

        engine_speed = self.engine.angular_velocity
        wheel_speeds = (
            left_wheel.angular_velocity,
            right_wheel.angular_velocity,
        )

        gearbox_speed = self._calculate_gearbox_speed_for_wheels(
            wheel_speeds
        )
        speed_mismatch = engine_speed - gearbox_speed

        if abs(speed_mismatch) <= 1e-12:
            return engine_speed, wheel_speeds

        # A change in engine speed changes the mismatch by correction / Ie.
        # Each wheel's correction changes gearbox input speed according to
        # the signed ratio and the differential's mean wheel speed.
        inverse_effective_inertia = (
            1.0 / self.engine.inertia
            + overall_ratio**2
            * (
                1.0 / left_wheel.inertia
                + 1.0 / right_wheel.inertia
            )
            / 4.0
        )

        correction = speed_mismatch / inverse_effective_inertia

        candidate_engine_speed = (
            engine_speed - correction / self.engine.inertia
        )

        if candidate_engine_speed < 0.0:
            return None

        candidate_left_speed = (
            wheel_speeds[0]
            + overall_ratio * correction / (2.0 * left_wheel.inertia)
        )
        candidate_right_speed = (
            wheel_speeds[1]
            + overall_ratio * correction / (2.0 * right_wheel.inertia)
        )

        return (
            candidate_engine_speed,
            (candidate_left_speed, candidate_right_speed),
        )


    def _calculate_wheel_acceleration(
        self,
        wheel: Wheel,
        drive_torque: float,
        angular_velocity: float,
    ) -> float:
        """Calculate wheel angular acceleration for a candidate state."""
        external_torque = self._calculate_external_wheel_torque(
            wheel,
            drive_torque,
            angular_velocity,
        )

        return (drive_torque + external_torque) / wheel.inertia


    def _set_driven_wheel_torques(self, clutch_torque: float) -> None:
        """Apply clutch torque through the gearbox and differential."""
        gearbox_output_torque = self.gearbox.calculate_output_torque(
            -clutch_torque
        )

        differential_input_torque = (
            gearbox_output_torque
            * self.final_drive_ratio
            * self.final_drive_efficiency
        )

        left_drive_torque, right_drive_torque = (
            self.differential.calculate_output_torques(
                differential_input_torque
            )
        )

        self.driven_wheels[0].set_drive_torque(left_drive_torque)
        self.driven_wheels[1].set_drive_torque(right_drive_torque)


    def step(self, dt: float) -> None:
        """
        Advance engine and driven-wheel rotation by one timestep.

        The solver first evaluates whether the clutch can lock. If locking
        fails at any stage, all calculations for the slipping mode use the
        original speeds; a rejected synchronisation is never applied.
        """
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive.")

        self.gearbox_input_angular_velocity = (
            self.calculate_gearbox_input_speed()
        )

        gear_ratio = self.gearbox.gear_ratio

        # Neutral disconnects engine torque from the driven wheels.
        if gear_ratio is None:
            self.clutch_torque = 0.0

            engine_net_torque = self.engine.calculate_net_torque()
            next_engine_speed = (
                self.engine.angular_velocity
                + engine_net_torque / self.engine.inertia * dt
            )
            self.engine.angular_velocity = max(0.0, next_engine_speed)

            for wheel in self.driven_wheels:
                wheel.set_drive_torque(0.0)
                wheel.integrate_angular_velocity(dt)

            self.gearbox_input_angular_velocity = (
                self.calculate_gearbox_input_speed()
            )
            return

        # Preserve the original speeds until the clutch mode is decided.
        original_engine_speed = self.engine.angular_velocity
        original_wheel_speeds = (
            self.driven_wheels[0].angular_velocity,
            self.driven_wheels[1].angular_velocity,
        )

        # Engine torque is evaluated at the start of the timestep. The
        # synchronisation correction is an instantaneous speed adjustment;
        # engine torque then acts over the timestep that follows.
        engine_free_torque = self.engine.calculate_net_torque()

        original_required_torque = self._calculate_locked_clutch_torque(
            engine_free_torque=engine_free_torque,
            gear_ratio=gear_ratio,
            wheel_speeds=original_wheel_speeds,
        )

        # Torque only can_lock check.
        slip_speed = self.clutch.calculate_slip_speed(
            original_engine_speed,
            self.gearbox_input_angular_velocity,
        )

        fully_engaged = self.clutch.engagement >= 1.0

        can_lock = (
            self.clutch.torque_capacity > 0.0
            and abs(original_required_torque) <= self.clutch.torque_capacity
            and (
                fully_engaged
                or abs(slip_speed) <= self.clutch.lock_slip_threshold
            )
        )

        candidate_engine_speed = original_engine_speed
        candidate_wheel_speeds = original_wheel_speeds
        selected_clutch_torque = original_required_torque


        if can_lock:
            # Synchronise only in local candidate values. Nothing has been
            # changed on the engine or wheels at this point.
            speed_candidate = self._calculate_locked_speed_candidate()

            if speed_candidate is None:
                can_lock = False
            else:
                candidate_engine_speed, candidate_wheel_speeds = (
                    speed_candidate
                )

                candidate_gearbox_speed = (
                    self._calculate_gearbox_speed_for_wheels(
                        candidate_wheel_speeds
                    )
                )

                # Recalculate the torque needed after synchronisation.
                # The engine's free torque remains evaluated at the start
                # of the timestep, avoiding a premature state mutation.
                selected_clutch_torque = (
                    self._calculate_locked_clutch_torque(
                        engine_free_torque=engine_free_torque,
                        gear_ratio=gear_ratio,
                        wheel_speeds=candidate_wheel_speeds,
                    )
                )

                can_lock = self.clutch.is_locked(
                    engine_angular_velocity=candidate_engine_speed,
                    gearbox_angular_velocity=candidate_gearbox_speed,
                    required_engine_torque=selected_clutch_torque,
                )


                if can_lock:
                    # Check that the locked engine will not cross below zero
                    # during integration. If it would, use the slipping mode
                    # rather than breaking the speed constraint by clamping
                    # the engine independently.
                    gearbox_output_torque = self.gearbox.calculate_output_torque(
                        -selected_clutch_torque
                    )

                    differential_input_torque = (
                        gearbox_output_torque
                        * self.final_drive_ratio
                        * self.final_drive_efficiency
                    )

                    left_drive_torque, right_drive_torque = (
                        self.differential.calculate_output_torques(
                            differential_input_torque
                        )
                    )

                    engine_net_torque = (
                        engine_free_torque + selected_clutch_torque
                    )

                    predicted_engine_speed = (
                        candidate_engine_speed
                        + engine_net_torque / self.engine.inertia * dt
                    )

                    predicted_left_speed = (
                        candidate_wheel_speeds[0]
                        + self._calculate_wheel_acceleration(
                            self.driven_wheels[0],
                            left_drive_torque,
                            candidate_wheel_speeds[0],
                        )
                        * dt
                    )

                    predicted_right_speed = (
                        candidate_wheel_speeds[1]
                        + self._calculate_wheel_acceleration(
                            self.driven_wheels[1],
                            right_drive_torque,
                            candidate_wheel_speeds[1],
                        )
                        * dt
                    )

                    # For a locked clutch, engine speed must equal the
                    # gearbox ratio times the mean driven-wheel speed.
                    predicted_gearbox_speed = (
                        gear_ratio
                        * self.final_drive_ratio
                        * (predicted_left_speed + predicted_right_speed)
                        / 2.0
                    )

                    if (
                        predicted_engine_speed < 0.0
                        or not math.isclose(
                            predicted_engine_speed,
                            predicted_gearbox_speed,
                            rel_tol=1e-7,
                            abs_tol=1e-7,
                        )
                    ):
                        can_lock = False


        if can_lock:
            # Commit the synchronised speeds only after the lock decision
            # and predicted integration have both succeeded.
            self.engine.angular_velocity = predicted_engine_speed

            self.driven_wheels[0].angular_velocity = predicted_left_speed
            self.driven_wheels[1].angular_velocity = predicted_right_speed

            self.clutch_torque = selected_clutch_torque
            self._set_driven_wheel_torques(self.clutch_torque)


        else:
            # The clutch did not lock. Use the untouched original speeds
            # and the original torque requirement for the slipping solution.
            self.engine.angular_velocity = original_engine_speed

            for wheel, original_speed in zip(
                self.driven_wheels,
                original_wheel_speeds,
            ):
                wheel.angular_velocity = original_speed

            self.clutch_torque = self.clutch.calculate_transmitted_torque(
                engine_angular_velocity=original_engine_speed,
                gearbox_angular_velocity=self.gearbox_input_angular_velocity,
                required_engine_torque=original_required_torque,
            )

            self._set_driven_wheel_torques(self.clutch_torque)

            engine_net_torque = self.engine.calculate_net_torque(
                clutch_torque=self.clutch_torque
            )

            next_engine_speed = (
                self.engine.angular_velocity
                + engine_net_torque / self.engine.inertia * dt
            )
            self.engine.angular_velocity = max(0.0, next_engine_speed)

            for wheel in self.driven_wheels:
                wheel.integrate_angular_velocity(dt)

        # Keep the reported gearbox input speed aligned with the final state.
        self.gearbox_input_angular_velocity = (
            self.calculate_gearbox_input_speed()
        )
