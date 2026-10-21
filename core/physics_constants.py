"""
Shared physical constants and default environemnt values
"""

# Gravitational acceleration at Earth's surface (m/s^2).
GRAVITY: float = 9.81

# Standard sea-level air density (kg/m^3).
AIR_DENSITY: float = 1.225

# Mass of the driver
DRIVER_MASS: float = 82.0

# Fuel Mass per litre, kg.
PETROL_MASS_PER_LITRE: float = 0.745
DIESEL_MASS_PER_LITRE: float = 0.845


# INITIAL SURFACE GRIP PRESETS
# These are tuning multipliers, not universal measured friction coefficients.
DRY_ASPHALT: float = 1.0
WET_ASPHALT: float = 0.70
LOOSE_GRAVEL: float = 0.65
ICE: float = 0.15
