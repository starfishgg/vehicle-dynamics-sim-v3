import math

from core.magic_formulas import magic_formula
from tyres.tyre_parameters import TyreParameters




class Tyre:
    """Calculate longitudinal and lateral tyre forces."""

    def __init__(self, parameters: TyreParameters) -> None:
        self.parameters = parameters


    @staticmethod
    def _validate_input(name: str, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")


    @staticmethod
    def _validate_normal_load(normal_load: float) -> None:
        if not math.isfinite(normal_load) or normal_load < 0.0:
            raise ValueError("normal_load must be finite and non-negative.")


    def calculate_longitudinal_force(
            self,
            slip_ratio: float,
            normal_load: float,
    ) -> float:
        """Calculate forward/backward tyre force in newtons."""
        self._validate_input("slip_ratio", slip_ratio)
        self._validate_normal_load(normal_load)

        response = magic_formula(
            slip=slip_ratio,
            stiffness=self.parameters.longitudinal_stiffness,
            shape=self.parameters.longitudinal_shape,
            peak=self.parameters.longitudinal_peak,
            curvature=self.parameters.longitudinal_curvature,
        )

        return (
            normal_load
            * self.parameters.friction_coefficient
            * response
        )


    def calculate_lateral_force(
            self,
            slip_angle: float,
            normal_load: float,
    ) -> float:
        """
        Calculate left/right tyre force in newtons.
        
        Positive slip angle represents the wheel pointing left relative
        to its direction of travel, producing positive lateral force.
        """
        self._validate_input("slip_angle", slip_angle)
        self._validate_normal_load(normal_load)

        response = magic_formula(
            slip=slip_angle,
            stiffness=self.parameters.lateral_stiffness,    # B
            shape=self.parameters.lateral_shape,            # C
            peak=self.parameters.lateral_peak,              # D
            curvature=self.parameters.lateral_curvature,    # E
        )

        return (
            normal_load
            * self.parameters.friction_coefficient          # grip_coefficient
            * response
        )

    @staticmethod
    def calculate_reaction_torque(
        longitudinal_force: float,
        wheel_radius: float,
    ) -> float:
        """Calculate the road's reaction torque on the wheel in Nm."""
        if not math.isfinite(longitudinal_force):
            raise ValueError("longitudinal_force must be finite.")

        if not math.isfinite(wheel_radius) or wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be finite and positive.")

        return -longitudinal_force * wheel_radius