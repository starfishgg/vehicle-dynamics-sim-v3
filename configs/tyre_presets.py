"""
Reusable tyre parameter presets.
"""

from tyres.tyre import Tyre
from tyres.tyre_parameters import TyreParameters




def create_road_tyre() -> Tyre:
    """
    Create a general-purpose road tyre.
    """
    parameters = TyreParameters(
        friction_coefficient=0.90,      # grip_coefficient

        longitudinal_stiffness=10.0,    # B
        longitudinal_shape=1.9,         # C
        longitudinal_peak=1.0,          # D
        longitudinal_curvature=0.97,    # E

        lateral_stiffness=5.0,          # B
        lateral_shape=1.3,              # C
        lateral_peak=1.0,               # D
        lateral_curvature=-1.6,         # E
    )
    return Tyre(parameters)


def create_sports_tyre() -> Tyre:
    """Create a higher-grip tyre for sporty road cars."""
    parameters = TyreParameters(
            friction_coefficient=1.10,      # grip_coefficient

            longitudinal_stiffness=10.0,    # B
            longitudinal_shape=1.9,         # C
            longitudinal_peak=1.0,          # D
            longitudinal_curvature=0.97,    # E
            
            lateral_stiffness=7.0,          # B
            lateral_shape=1.3,              # C
            lateral_peak=1.0,               # D
            lateral_curvature=-1.6,         # E
    )
    return Tyre(parameters)


def create_track_tyre() -> Tyre:
    """Create a high-grip tyre for track-focused cars."""
    parameters = TyreParameters(
            friction_coefficient=1.30,      # grip_coefficient

            longitudinal_stiffness=12.0,    # B
            longitudinal_shape=1.9,         # C
            longitudinal_peak=1.0,          # D
            longitudinal_curvature=0.97,    # E
            
            lateral_stiffness=9.0,          # B
            lateral_shape=1.3,              # C
            lateral_peak=1.0,               # D
            lateral_curvature=-1.6,         # E
    )
    return Tyre(parameters)

