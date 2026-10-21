import math




def torque_curve(
        rpm: float,
        peak_torque: float,
        base_torque: float,
        peak_rpm: float,
        rise_width: float,
        fall_width: float,
        ) -> float:
    """
    Calculate engine torque at a given RPM.

    The curve uses two Gaussian widths:

    - rise_width controls the shape before peak torque
    - fall_width controls the shape after peak torque

    This allows the engine to have a different torque curve on
    either side of its peak.

    Args:
        rpm:
            Current engine speed in revolutions per minute.

        peak_torque:
            Maximum torque produced by the engine in Nm.

        base_torque:
            Minimum torque level used by the curve in Nm.

        peak_rpm:
            RPM where peak torque occurs.

        rise_width:
            Width of the curve before peak RPM.

        fall_width:
            Width of the curve after peak RPM.

    Returns:
        Estimated engine torque in Nm.
    """

    # Use a different curve width depending on whether the engine
    # is below or above its peak torque RPM.
    if rpm < peak_rpm:
        width = rise_width
    else:
        width = fall_width

    torque_difference = peak_torque - base_torque
    distance_from_peak = rpm - peak_rpm

    torque = (
        torque_difference
        * math.exp(
            -(distance_from_peak**2) / (2.0 * width**2)
        )
        + base_torque
    )

    return torque




def magic_formula(
    slip: float,        # grip_coefficient
    stiffness: float,   # B
    shape: float,       # C
    peak: float,        # D
    curvature: float,   # E
) -> float:
    """Calculate a Magic Formula response for a given slip value.

    The result is a dimensionless curve response when peak is dimensionless.
    Actual tyre force can be calculated by scaling this response by the
    normal load and friction coefficient.
    """

    values = (slip, stiffness, shape, peak, curvature)

    if not all(math.isfinite(value) for value in values):
        raise ValueError("Magic Formula inputs must be finite.")

    if stiffness <= 0.0:
        raise ValueError("stiffness must be positive.")

    if shape <= 0.0:
        raise ValueError("shape must be positive.")

    if peak < 0.0:
        raise ValueError("peak cannot be negative.")

    bx = stiffness * slip
    angle = bx - curvature * (bx - math.atan(bx))

    return peak *  math.sin(shape * math.atan(angle))

