import math

from core.magic_formulas import torque_curve




class Engine:
    """
    Engine torque model.

    Calculates combustion torque and friction losses, but does not
    integrate engine speed. The drivetrain solver owns that calculation.

    Angular velocity is stored in rad/s. RPM is derived from it.
    """

    def __init__(
            self,
            *,
            peak_torque: float,
            base_torque: float,
            peak_torque_rpm: float,
            rise_width: float,
            fall_width: float,
            idle_rpm: float,
            redline_rpm: float,
            inertia: float,
            friction_torque: float = 22.0,
            friction_torque_per_rad_s: float = 0.0,
            initial_rpm: float | None = None,
    ) -> None:
        self._validate_configuration(
            peak_torque=peak_torque,
            base_torque=base_torque,
            peak_torque_rpm=peak_torque_rpm,
            rise_width=rise_width,
            fall_width=fall_width,
            idle_rpm=idle_rpm,
            redline_rpm=redline_rpm,
            inertia=inertia,
            friction_torque=friction_torque,
            friction_torque_per_rad_s=friction_torque_per_rad_s,
        )

        self.peak_torque = peak_torque
        self.base_torque = base_torque
        self.peak_torque_rpm = peak_torque_rpm
        self.rise_width = rise_width
        self.fall_width = fall_width

        self.idle_rpm = idle_rpm
        self.redline_rpm = redline_rpm
        self.inertia = inertia

        self.friction_torque = friction_torque
        self.friction_torque_per_rad_s = friction_torque_per_rad_s

        starting_rpm = idle_rpm if initial_rpm is None else initial_rpm
        self.angular_velocity = self._rpm_to_angular_velocity(starting_rpm)
        self.throttle = 0.0


    @staticmethod
    def _validate_configuration(
        *,
        peak_torque: float,
        base_torque: float,
        peak_torque_rpm: float,
        rise_width: float,
        fall_width: float,
        idle_rpm: float,
        redline_rpm: float,
        inertia: float,
        friction_torque: float = 22.0,
        friction_torque_per_rad_s: float = 0.0,
    ) -> None:
        values = {
            "peak_torque": peak_torque,
            "base_torque": base_torque,
            "peak_torque_rpm": peak_torque_rpm,
            "rise_width": rise_width,
            "fall_width": fall_width,
            "idle_rpm": idle_rpm,
            "redline_rpm": redline_rpm,
            "inertia": inertia,
            "friction_torque": friction_torque,
            "friction_torque_per_rad_s": friction_torque_per_rad_s,
        }

        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")

        if peak_torque <= 0.0:
            raise ValueError("peak_torque must be positive.")

        if not 0.0 <= base_torque <= peak_torque:
            raise ValueError("base_torque must be between zero and peak_torque.")

        if rise_width <= 0.0 or fall_width <= 0.0:
            raise ValueError("Torque curve widths must be positive.")

        if idle_rpm <= 0.0:
            raise ValueError("idle_rpm must be positive.")

        if not idle_rpm <= peak_torque_rpm < redline_rpm:
            raise ValueError("peak_torque_rpm must be at least idle_rpm and below redline_rpm.")

        if inertia <= 0.0:
            raise ValueError("inertia must be positive.")

        if friction_torque < 0.0:
            raise ValueError("friction_torque cannot be negative.")

        if friction_torque_per_rad_s < 0.0:
            raise ValueError(
                "friction_torque_per_rad_s cannot be negative."
            )

    
    @staticmethod
    def _rpm_to_angular_velocity(rpm: float) -> float:
        if not math.isfinite(rpm) or rpm < 0.0:
            raise ValueError("RPM must be finite and non-negative.")

        return rpm * (2.0 * math.pi / 60.0)

    @staticmethod
    def _angular_velocity_to_rpm(angular_velocity: float) -> float:
        return angular_velocity * (60.0 / (2.0 * math.pi))


    @property
    def rpm(self) -> float:
        """Current engine speed in revolutions per minute."""
        return self._angular_velocity_to_rpm(self.angular_velocity)

    
    @property
    def angular_velocity(self) -> float:
        """Current engine angular velocity in rad/s."""
        return self._angular_velocity


    @angular_velocity.setter
    def angular_velocity(self, value: float) -> None:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("angular_velocity must be finite and non-negative.")

        self._angular_velocity = float(value)


    @property
    def throttle(self) -> float:
        return self._throttle


    @throttle.setter
    def throttle(self, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("Throttle must be finite.")

        self._throttle = max(0.0, min(value, 1.0))


    def set_throttle(self, throttle: float) -> None:
        """Set throttle input, clamped to the range 0.0 to 1.0."""
        self.throttle = throttle


    def calculate_combustion_torque(self) -> float:
        """Return throttle-dependent engine torque before losses."""

        if self.rpm >= self.redline_rpm:
            return 0.0

        full_throttle_torque = torque_curve(
            rpm=self.rpm,
            peak_torque=self.peak_torque,
            base_torque=self.base_torque,
            peak_rpm=self.peak_torque_rpm,
            rise_width=self.rise_width,
            fall_width=self.fall_width,
        )

        return full_throttle_torque * self.throttle


    def calculate_friction_torque(self) -> float:
        """Return the singed torque caused by internal engine friction.
        
        Friction opposes forward engine rotation, so this torque is
        negative while the engine is turning and zero at rest.
        """
        if self.angular_velocity == 0.0:
            return 0.0

        friction_magnitude = (
            self.friction_torque
            + self.friction_torque_per_rad_s * self.angular_velocity
        )

        return -friction_magnitude


    def calculate_net_torque(self, clutch_torque: float = 0.0) -> float:
        """Return net engine torque without changing engine speed.

        clutch_torque is the signed torque applied to the engine by
        the clutch. A clutch resisting forward engine rotation applies
        negative torque.
        """
        if not math.isfinite(clutch_torque):
            raise ValueError("clutch_torque must be finite.")

        return (
            self.calculate_combustion_torque()
            + self.calculate_friction_torque()
            + clutch_torque
        )