import math




class Suspension:
    """
    Spring-and-damper model for one suspension corner.

    Units:
        spring_rate: N/m
        damping_rate: N s/m
        compression: m
        compression_velocity: m/s
        force: N

    Positive compression means the suspension is compressed.
    Positive compression velocity means further compression.
    The returned force acts upward on the chassis.
    """

    def __init__(
            self,
            *,
            spring_rate: float,
            damping_rate: float,
    ) -> None:
        if not math.isfinite(spring_rate) or spring_rate <= 0.0:
            raise ValueError("spring_rate must be finite and positive.")

        if not math.isfinite(damping_rate) or damping_rate < 0.0:
            raise ValueError("damping_rate must be finite and non-negative.")

        self.spring_rate = spring_rate
        self.damping_rate = damping_rate


    def calculate_force(
            self,
            compression: float,
            compression_velocity: float,
    ) -> float:
        """
        Calculate the suspension force acting upward on the chassis.

        Spring force increases with compression. Damper force opposes
        suspension movement. The total force is clamped at zero so
        this simple model cannot generate a tensile suspension force.
        """
        if not math.isfinite(compression):
            raise ValueError("compression must be finite.")

        if not math.isfinite(compression_velocity):
            raise ValueError("compression_velocity must be finite.")

        spring_force = self.spring_rate * compression
        damping_force = self.damping_rate * compression_velocity

        return max(0.0, spring_force + damping_force)

    