import math



class Gearbox:
    """
    Gearbox with selectable forward gears, neutral and reverse.

    Gear numbering:
        -1: reverse
         0: neutral
         1+: forward gears

    Ratios are defined as input speed / output speed. A negative
    ratio indicates reverse rotation.

    The gearbox calculates speed and torque relationships but does
    not integrate either shaft's dynamics or manage shift timing.
    """

    def __init__(
            self,
            *,
            forward_ratios: tuple[float, ...],
            reverse_ratio: float,
            efficiency: float = 0.97,
            initial_gear: int = 0,
    ) -> None:
        self._validate_configuration(
            forward_ratios=forward_ratios,
            reverse_ratio=reverse_ratio,
            efficiency=efficiency,
        )

        self.forward_ratios = tuple(forward_ratios)
        self.reverse_ratio = -reverse_ratio
        self.efficiency = efficiency
        #self.selected_gear = 0

        self.select_gear(initial_gear)

    @staticmethod
    def _validate_configuration(
        *,
        forward_ratios: tuple[float, ...],
        reverse_ratio: float,
        efficiency: float,
    ) -> None:
        if not forward_ratios:
            raise ValueError("At least one forward gear is required.")

        for index, ratio in enumerate(forward_ratios, start=1):
            if not math.isfinite(ratio) or ratio <= 0.0:
                raise ValueError(f"Forward gear {index} ratio must be finite and positive.")

        if not math.isfinite(reverse_ratio) or reverse_ratio <= 0.0:
            raise ValueError("reverse_ratio must be finite and positive.")

        if (
            not math.isfinite(efficiency)
            or not 0.0 < efficiency <= 1.0
        ):
            raise ValueError("efficiency must be in the range (0.0, 1.0].")

        
    @property
    def gear_count(self) -> int:
        """Number of available forward gears."""
        return len(self.forward_ratios)

    @property
    def gear_ratio(self) -> float | None:
        """Seleected ratio, or None when neutral is selected."""
        if self.selected_gear == 0:
            return None

        if self.selected_gear == -1:
            return self.reverse_ratio

        return self.forward_ratios[self.selected_gear -1]

    
    @property
    def is_neutral(self) -> bool:
        return self.selected_gear == 0


    def select_gear(self, gear: int) -> None:
        """Select reverse, neutral or an available forward gear."""
        if isinstance(gear, bool) or not isinstance(gear, int):
            raise ValueError("Gear selection must be an integer.")

        if gear < -1 or gear > self.gear_count:
            raise ValueError(f"Gear {gear} is invalid. Select reverse (-1), neutral (0), or a forward gear from 1 to {self.gear_count}.")

        self.selected_gear = gear


    def calculate_output_angular_velocity(
            self,
            input_angular_velocity: float,
    ) -> float | None:
        """
        Calculate output angulary velocity in rad/s.
        
        Returns None in neutral because the gearbox does not constrain
        the output shaft speed when no gear is engaged.
        """

        if not math.isfinite(input_angular_velocity):
            raise ValueError("Input angulary velocity must be finite.")

        ratio = self.gear_ratio

        if ratio is None:
            return None

        return input_angular_velocity / ratio


    def calculate_output_torque(self, input_torque: float) -> float:
        """
        Calculate output torque in Nm for input-driven power flow.

        Neutral transmits no torque. Efficiency accounts for drivetrain
        losses. Reverse has a negative ratio, reversing output torque.
        """
        if not math.isfinite(input_torque):
            raise ValueError("Input torque must be finite.")

        ratio = self.gear_ratio

        if ratio is None:
            return 0.0

        return input_torque * ratio * self.efficiency


    def calculate_input_angular_velocity(
            self,
            output_angular_velocity: float,
    ) -> float | None:
        """
        Calculate the input speed required by the selected ratio.
        
        Returns None in neutral.
        """
        if not math.isfinite(output_angular_velocity):
            raise ValueError("Output angular velocity must be finite.")

        ratio = self.gear_ratio

        if ratio is None:
            return None

        return output_angular_velocity * ratio
    
