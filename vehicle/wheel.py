import math




class Wheel:
    """
    Wheel rotational state and torque calculations.

    Angular velocity is signed and measured in rad/s.
    Torque is measured in Nm, inertia in kg m^2, and radius in metres.

    Drive torque and tyre reaction torque are signed external torques.
    Brake input is normalized from 0.0 (off) to 1.0 (maximum).

    This class does not calculate tyre grip, suspension loads or
    chassis motion.
    """

    def __init__(
            self,
            *,
            radius: float,
            inertia: float,
            max_brake_torque: float,
            angular_velocity: float = 0.0,
    ) -> None:
        self._validate_positive("radius", radius)
        self._validate_positive("inertia", inertia)
        self._validate_positive("max_brake_torque", max_brake_torque)

        self.radius = radius
        self.inertia = inertia
        self.max_brake_torque = max_brake_torque

        self.angular_velocity = angular_velocity
        self.drive_torque = 0.0
        self.tyre_reaction_torque = 0.0
        self.brake_input = 0.0


    @staticmethod
    def _validate_positive(name: str, value: float) -> None:
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive.")


    @property
    def angular_velocity(self) -> float:
        """Current singed angular velocity in rad/s."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("angular_velocity must be finite.")

        self._angular_velocity = float(value)


    @property
    def rotational_speed_rpm(self) -> float:
        """Signed rotational speed in revolutions per minute."""
        return self.angular_velocity * (60.0 / (2.0 * math.pi))


    @property
    def linear_speed(self) -> float:
        """Curcumferential speed at the wheel radius, in m/s."""
        return self.angular_velocity * self.radius


    def set_drive_torque(self, torque: float) -> None:
        """Set signed torque supplied by the drivetrain."""
        if not math.isfinite(torque):
            raise ValueError("Drive torque must be finite.")

        self.drive_torque = torque


    def set_tyre_reaction_torque(self, torque: float) -> None:
        """Set signed torque exerted by the tyre on the wheel."""
        if not math.isfinite(torque):
            raise ValueError("Tyre reaction torque must be finite.")


        self.tyre_reaction_torque = torque


    def set_brake_input(self, brake_input: float) -> None:
        """Set normalised brake input, clamped to 0.0-1.0"""
        if not math.isfinite(brake_input):
            raise ValueError("Brake input must be finite.")

        self.brake_input = max(0.0, min(brake_input, 1.0))


    @property
    def brake_torque_capacity(self) -> float:
        """Available braking torque magnitude in Nm."""
        return self.brake_input * self.max_brake_torque


    def calculate_brake_torque(self) -> float:
        """
        Calculate signed brake toruq opposing wheel rotation.
        
        At rest, the brake opposes the combined drive and tyre torque
        up to its capacity. This lets the brake hold the wheel still
        when the applied torque is not strong enough to overcome it.
        """
        capacity = self.brake_torque_capacity

        if capacity == 0.0:
            return 0.0

        if self.angular_velocity > 0.0:
            return -capacity

        if self.angular_velocity < 0.0:
            return capacity

        external_torque = self.drive_torque + self.tyre_reaction_torque

        if external_torque == 0.0:
            return 0.0

        opposing_torque = -math.copysign(
            min(abs(external_torque), capacity),
            external_torque,
        )

        return opposing_torque


    def calculate_net_torque(self) -> float:
        """Calculate total signed torque acting on the wheel."""
        return (
            self.drive_torque
            + self.tyre_reaction_torque
            + self.calculate_brake_torque()
        )


    def calculate_angular_acceleration(self) -> float:
        """Calculate angular acceleration in rad/s^2."""
        return self.calculate_net_torque() / self.inertia


    def integrate_angular_velocity(self, dt: float) -> None:
        """
        Advance wheel speed by one timestep.

        The coupled drivetrain solver should eventually coordinate
        integration of driven wheel with engine and clutch dynamics.
        """
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive.")

        previous_velocity = self.angular_velocity
        acceleration = self.calculate_angular_acceleration()
        next_velocity = previous_velocity + acceleration * dt

        # Avoid numerical sign-flipping when braking brings the wheel
        # to a stop within this timestep.
        brake_torque = self.calculate_brake_torque()
        if (
            previous_velocity != 0.0
            and previous_velocity * next_velocity < 0.0
            and previous_velocity * brake_torque < 0.0
        ):
            next_velocity = 0.0

        self.angular_velocity = next_velocity

    