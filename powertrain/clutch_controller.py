"""
Translate clutch-pedal input into clutch engagement.
"""

import math

from powertrain.clutch import Clutch




class ClutchController:
    """Control clutch engagement from a normalised pedal input."""

    def __init__(self, clutch: Clutch) -> None:
        self.clutch = clutch

    def update(self, clutch_pedal: float) -> None:
        """
        Apply clutch-pedal input.

        0.0 means the pedal is released.
        1.0 means the pedal is fully pressed.
        """
        if not math.isfinite(clutch_pedal):
            raise ValueError("clutch_pedal must be finite.")

        clutch_pedal = max(0.0, min(clutch_pedal, 1.0))
        self.clutch.engagement = 1.0 - clutch_pedal
