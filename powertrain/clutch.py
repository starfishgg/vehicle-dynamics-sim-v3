import math




class Clutch:
    """
    Model torque transfer between the engine and gearbox.

    Engagement is between 0.0 (disengaged) and 1.0 (fully engaged).

    The clutch calculates torque transfer but does not update either
    shaft's angular velocity. The drivetrain solver owns that work.

    Torque returned by calculate_transmitted_torque() is the torque
    applied to the engine side of the clutch. The gearbox receives
    the equal and opposite torque.
    """

    def __init__(
            self,
            *,
            max_torque: float,
            engagement: float = 0.0,
            lock_slip_threshold: float = 0.5,
    ) -> None:
        self._validate_configuration(
            max_torque=max_torque,
            lock_slip_threshold=lock_slip_threshold,
        )

        self.max_torque = max_torque
        self.lock_slip_threshold = lock_slip_threshold
        self._engagement = 0.0
        self.engagement = engagement


    @staticmethod
    def _validate_configuration(
        *,
        max_torque: float,
        lock_slip_threshold: float,
    ) -> None:
        if not math.isfinite(max_torque) or max_torque <= 0.0:
            raise ValueError("max_torque must be finite and positive.")

        if (
            not math.isfinite(lock_slip_threshold)
            or lock_slip_threshold < 0.0
        ):
            raise ValueError("lock_slip_threshold must be finite and non-negative.")


    @property
    def engagement(self) -> float:
        """Current clutch engagement from 0.0 to 1.0."""
        return self._engagement

    @engagement.setter
    def engagement(self, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("Clutch engagement must be finite.")

        self._engagement = max(0.0, min(value, 1.0))

    
    @property
    def torque_capacity(self) -> float:
        """Maximum torque magnitude transmissible at current engagement."""
        return self.max_torque * self.engagement

    
    @staticmethod
    def calculate_slip_speed(
        engine_angular_velocity: float,
        gearbox_angular_velocity: float,
    ) -> float:
        """Return engine speed minus gearbox-input speed, in rad/s."""
        if (
            not math.isfinite(engine_angular_velocity)
            or not math.isfinite(gearbox_angular_velocity)
        ):
            raise ValueError("Angular velocities must be finite.")

        return engine_angular_velocity - gearbox_angular_velocity


    def is_locked(
            self,
            engine_angular_velocity: float,
            gearbox_angular_velocity: float,
            required_engine_torque: float = 0.0,
    ) -> bool:
        """
        Return whether the clutch can keep both sides rotating together.

        required_engine_torque is the torque the solver needs the clutch
        to apply to the engine side to maintain the coupled motion.
        """
        if not math.isfinite(required_engine_torque):
            raise ValueError("required_engine_torque must be finite.")

        if self.torque_capacity <= 0.0:
            return False

        slip_speed = self.calculate_slip_speed(
            engine_angular_velocity,
            gearbox_angular_velocity,
        )

        return (
            abs(slip_speed) <= self.lock_slip_threshold
            and abs(required_engine_torque) <= self.torque_capacity
        )


    def calculate_transmitted_torque(
            self,
            engine_angular_velocity: float,
            gearbox_angular_velocity: float,
            required_engine_torque: float = 0.0,
    ) -> float:
        """
        Return clutch torque applied to the engine.

        A locked clutch supplies the torque required to maintain the
        speed relationship. A slipping clutch opposes relative motion
        and cannot transmit more than its current torque capacity.
        """
        if not math.isfinite(required_engine_torque):
            raise ValueError("required_engine_torque must be finite.")

        capacity = self.torque_capacity

        if capacity <= 0.0:
            return 0.0

        slip_speed = self.calculate_slip_speed(
            engine_angular_velocity,
            gearbox_angular_velocity,
        )

        # Near zero slip, use the required torque direction. This avoids
        # tiny speed differences reversing the intended torque direction.
        if abs(slip_speed) <= self.lock_slip_threshold:
            return max(
                -capacity,
                min(capacity, required_engine_torque),
            )

        # During esablished slip, friction opposes relative rotation.
        return -math.copysign(capacity, slip_speed)


