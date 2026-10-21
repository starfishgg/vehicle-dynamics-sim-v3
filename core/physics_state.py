"""Vehicle-level physical state."""



class PhysicsState:
    """
    State of the vehicle body, expressed in consistent coordinates.

    World frame:
        X and Y describe position in the horizontal world plane.
        Z points upward.

    Body frame:
        Longitudinal velocity is positive forwards.
        Lateral velocity is positive to the left.

    Heading and yaw rate are positive counter-clockwise when viewed
    from above. Heading is measured in radians.

    Component-specific state, such as engine RPM and wheel angular
    velocity, remains owned by those respective components.
    """

    def __init__(self):
        # World position, metres.
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0

        # Vehicle orientation, radians.
        self.heading = 0.0
        self.yaw_rate = 0.0

        # Velocity in the vehicle's body frame, metres per second.
        self.longitudinal_velocity = 0.0
        self.lateral_velocity = 0.0
        self.vertical_velocity = 0.0

        # Angular velocity about body axes, radians per second.
        self.roll_rate = 0.0
        self.pitch_rate = 0.0


        # Accelerations, metres per second squared.
        self.longitudinal_acceleration = 0.0
        self.lateral_acceleration = 0.0
        self.vertical_acceleration = 0.0


    @property
    def speed(self) -> float:
        """Magnitude of velocity in the vehicle's body frame."""
        return(
            self.longitudinal_velocity**2
            + self.lateral_velocity**2
            + self.vertical_velocity**2
        ) ** 0.5
    