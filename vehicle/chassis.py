"""
Physical dimensions and mass properties of the vehicle.
"""

import math

from core.physics_constants import (
    GRAVITY, DRIVER_MASS, AIR_DENSITY, PETROL_MASS_PER_LITRE, DIESEL_MASS_PER_LITRE
)




class Chassis:
    """
    Vehicle-level mass, geometry and road-load parameters.
    """

    def __init__(
            self,
            *,
            dry_mass: float,
            wheelbase: float,
            front_track: float,
            rear_track: float,
            centre_of_gravity_height: float,
            fuel_tank_capacity_litres: float,
            initial_fuel_litres: float | None = None,
            fuel_density: float = PETROL_MASS_PER_LITRE,
            driver_mass: float = DRIVER_MASS,
            front_weight_distribution: float = 0.55,
            drag_coefficient: float = 0.30,
            frontal_area: float = 2.10,
            rolling_resistance_coefficient: float = 0.012,
    ) -> None:
        positive_values = {
            "dry_mass": dry_mass,
            "wheelbase": wheelbase,
            "front_track": front_track,
            "rear_track": rear_track,
            "centre_of_gravity_height": centre_of_gravity_height,
            "fuel_tank_capacity_litres": fuel_tank_capacity_litres,
            "initial_fuel_litres": initial_fuel_litres,
            "fuel_density": fuel_density,
            "frontal_area": frontal_area,
        }

        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} mmust be finite and positive.")

        non_negative_values = {
            "drag_coefficient": drag_coefficient,
            "rolling_resistance_coefficient": rolling_resistance_coefficient,
        }

        for name, value in non_negative_values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")

        if (
            not math.isfinite(front_weight_distribution)
            or not 0.0 < front_weight_distribution < 1.0
        ):
            raise ValueError("front_weight_distribution must be between 0 and 1.")

        if initial_fuel_litres is None:
            initial_fuel_litres = fuel_tank_capacity_litres

        if (
            not math.isfinite(initial_fuel_litres)
            or not 0.0 <= initial_fuel_litres <= fuel_tank_capacity_litres
        ):
            raise ValueError("initial_fuel_mass must be between zero and fuel_tank_capacity.")

        self.dry_mass = dry_mass
        self.driver_mass = driver_mass

        self.fuel_tank_capacity_litres = fuel_tank_capacity_litres
        self._fuel_litres = initial_fuel_litres
        self.fuel_density = fuel_density

        self.wheelbase = wheelbase
        self.front_track = front_track
        self.rear_track = rear_track
        self.centre_of_gravity_height = centre_of_gravity_height
        self.front_weight_distribution = front_weight_distribution

        self.drag_coefficient = drag_coefficient
        self.frontal_area = frontal_area
        self.rolling_resistance_coefficient = rolling_resistance_coefficient


    @property
    def fuel_litres(self) -> float:
        """Current fuel volume in litres."""
        return self._fuel_litres

    @fuel_litres.setter
    def fuel_litres(self, value: float) -> None:
        if (
            not math.isfinite(value)
            or not 0.0 <= value <= self.fuel_tank_capacity_litres
        ):
            raise ValueError("fuel_liters must be between zero and tank capacity.")

        self._fuel_litres = value

    @property
    def fuel_mass(self) -> float:
        """Current fuel in kilograms."""
        return self.fuel_litres * self.fuel_density

    
    @property
    def fuel_level(self) -> float:
        """Remaining fuel as a percentage from 0 to 100."""
        return 100.0 * self.fuel_litres / self.fuel_tank_capacity_litres


    @property
    def total_mass(self) -> float:
        """Current vehicle, driver and fuel mass in kilograms."""
        return (
            self.dry_mass
            + self.driver_mass
            + self.fuel_mass
        )

    @property
    def rear_weight_distrubution(self) -> float:
        """Static proportion of vehicle weight carried by the rear axle."""
        return 1.0 - self.front_weight_distribution


    def consume_fuel(self, litres: float) -> None:
        """remove fuel mass in kilograms, stopping at an empty tank."""
        if not math.isfinite(litres) or litres < 0.0:
            raise ValueError("Fuel consumption must be finite and non-negative.")

        self.fuel_litres = max(0.0, self.fuel_litres - litres)
    

    def calculate_static_axle_loads(
            self,
            gravity: float = GRAVITY,
    ) -> tuple[float, float]:
        """Return front and read axle loads in newtons."""
        if not math.isfinite(gravity) or gravity <= 0.0:
            raise ValueError("gravity must be finite and positive.")

        total_weight = self.total_mass * gravity

        front_load = total_weight * self.front_weight_distribution
        rear_load = total_weight * self.rear_weight_distrubution

        return front_load, rear_load

    
    def calculate_dynamic_axle_loads(
            self,
            longitudinal_acceleration: float,
            gravity: float = GRAVITY,
    ) -> tuple[float, float]:
        """
        Calculate front and rear axle loads, including longitudinal
        weight transfer caused by acceleration or braking.

        Positive acceleration transfers load to the rear axle.
        Negative acceleration transfers load to the front axle.

        The total load remains equal to the vehicle's weight. Axle
        loads are prevented from becoming negative.
        """
        if not math.isfinite(longitudinal_acceleration):
            raise ValueError("longitudinal_acceleration must be finite.")

        if not math.isfinite(gravity) or gravity <= 0.0:
            raise ValueError("gravity must be finite and positive.")

        static_front_load, static_rear_load = (
            self.calculate_static_axle_loads(gravity)
        )

        load_transfer = (
            self.total_mass
            * longitudinal_acceleration
            * self.centre_of_gravity_height
            / self.wheelbase
        )

        # Limit the transfer so neither axle develops a negative load.
        load_transfer = max(
            -static_rear_load,
            min(load_transfer, static_front_load),
        )

        front_load = static_front_load - load_transfer
        rear_load = static_rear_load + load_transfer

        return front_load, rear_load

