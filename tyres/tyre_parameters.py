import math




class TyreParameters:
    """Configurable coefficients for a simplified tyre model."""

    def __init__(
        self,
        *,
        friction_coefficient: float = 1.0,      # grip_coefficient

        longitudinal_stiffness: float = 10.0,   # B
        longitudinal_shape: float = 1.9,        # C
        longitudinal_peak: float = 1.0,         # D
        longitudinal_curvature: float = 0.97,   # E
        
        lateral_stiffness: float = 5.0,         # B
        lateral_shape: float = 1.3,             # C
        lateral_peak: float = 1.0,              # D
        lateral_curvature: float = -1.6,        # E
    ) -> None:

        positive_values = {
            "friction_coefficient": friction_coefficient,

            "longitudinal_stiffness": longitudinal_stiffness,
            "longitudinal_shape": longitudinal_shape,
            
            "lateral_stiffness": lateral_stiffness,
            "lateral_shape": lateral_shape,
        }

        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")

        peak_values = {
            "longitudinal_peak": longitudinal_peak,
            "lateral_peak": lateral_peak,
        }

        for name, value in peak_values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")

        curvature_values = (
            longitudinal_curvature,
            lateral_curvature,
        )

        if not all(math.isfinite(value) for value in curvature_values):
            raise ValueError("Curvature values must be finite.")

        self.friction_coefficient = friction_coefficient

        self.longitudinal_stiffness = longitudinal_stiffness
        self.longitudinal_shape = longitudinal_shape
        self.longitudinal_peak = longitudinal_peak
        self.longitudinal_curvature = longitudinal_curvature

        self.lateral_stiffness = lateral_stiffness
        self.lateral_shape = lateral_shape
        self.lateral_peak = lateral_peak
        self.lateral_curvature = lateral_curvature
