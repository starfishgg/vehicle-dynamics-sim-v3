"""
Reusable engine configurations for vehicle setup.
"""

from powertrain.engine import Engine




def create_road_petrol_engine() -> Engine:
    """
    Create a conventional petrol engine for a road car.
    """
    return Engine(
        idle_rpm=750.0,
        peak_torque=250.0,
        base_torque=100.0,
        peak_torque_rpm=4000.0,
        rise_width=1600.0,
        fall_width=2400.0,
        redline_rpm=6500.0,
        inertia=0.18,
    )


def create_sporty_petrol_engine() -> Engine:
    """
    Create a higher-torque petrol engine for a sporty car.
    """
    return Engine(
        idle_rpm=850.0,
        peak_torque=400.0,
        base_torque=120.0,
        peak_torque_rpm=4500.0,
        rise_width=1800.0,
        fall_width=3000.0,
        redline_rpm=7000.0,
        inertia=0.15, 
    )


def create_high_revving_engine() -> Engine:
    """
    Create a high-revving engine for a performance car.
    """
    return Engine(
        idle_rpm=1000.0,
        peak_torque=500.0,
        base_torque=150.0,
        peak_torque_rpm=6000.0,
        rise_width=2200.0,
        fall_width=3000.0,
        redline_rpm=8500.0,
        inertia=0.12, 
    )