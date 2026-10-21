import math




class Differential:
    """
    Open differential connecting the two driven wheels of an axle.

    The differential allows the wheels to rotate at different speeds
    while applying equal torque to both sides.

    It calculates torque and speed relationships but does not update
    wheel angular velocities. The drivetrain solver owns integration.
    """

    def __init__(self, *, efficiency: float = 0.97) -> None:
        if not math.isfinite(efficiency) or not 0.0 < efficiency <= 1.0:
            raise ValueError("efficiency must be in the range (0.0, 1.0].")

        self.efficiency = efficiency


    @staticmethod
    def _validate_finite(**values: float) -> None:
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")


    def calculate_output_torques(
            self,
            input_torque: float,
    ) -> tuple[float, float]:
        """
        Return left and right wheel torque in Nm.
        
        
        Input torque is applied to the differential carrier. The
        differential divides the available torque equally between
        both wheels, accounting for its efficiency.
        """
        self._validate_finite(input_torque=input_torque)

        wheel_torque = input_torque * self.efficiency / 2.0
    
        return wheel_torque, wheel_torque


    def calculate_input_angular_velocity(
            self,
            left_wheel_angular_velocity: float,
            right_wheel_angular_velocity: float,
    ) -> float:
        """
        Return carrier angular velocity in rad/s.

        For a symmetric open differential, carrier speed is the
        arithmetic mean of the left and right wheel speeds.
        """
        self._validate_finite(
            left_wheel_angular_velocity=left_wheel_angular_velocity,
            right_wheel_angular_velocity=right_wheel_angular_velocity,
        )

        return (
            left_wheel_angular_velocity + right_wheel_angular_velocity
        ) / 2.0


    def calculate_right_wheel_angular_velocity(
            self,
            input_angular_velocity: float,
            left_wheel_angular_velocity: float,
    ) -> float:
        """Calculate right wheel speed from carrier and left wheel speed."""
        self._validate_finite(
            input_angular_velocity=input_angular_velocity,
            left_wheel_angular_velocity=left_wheel_angular_velocity,
        )

        return 2.0 * input_angular_velocity - left_wheel_angular_velocity


    def calculate_left_wheel_angular_velocity(
            self,
            input_angular_velocity: float,
            right_wheel_angular_velocity: float,
    ) -> float:
        """Calculate left wheel speed from carrier and right wheel speed."""
        self._validate_finite(
            input_angular_velocity=input_angular_velocity,
            right_wheel_angular_velocity=right_wheel_angular_velocity,
        )

        return 2.0 * input_angular_velocity - right_wheel_angular_velocity