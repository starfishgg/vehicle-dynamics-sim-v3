"""Normalised player input for one simulation step."""



class DriverInput:
    """
    Player controls, independent of input-device implementation.

    Throttle:       0.0 to 1.0
    Brake:          0.0 to 1.0
    Steering:      -1.0 to 1.0; positive means left
    Requested gear: None means no new request;
                    -1 means reverse;
                     0 means neutral;
                     1 or greater means a forward gear.

    Gear selection is a request, not an instruction to change the
    gearbox's current gear immediately.
    """

    def __init__(
            self,
            throttle: float = 0.0,
            brake: float = 0.0,
            steering: float = 0.0,
            requested_gear: int | None = None,
    ):
        self.throttle = self._clamp(throttle, 0.0, 1.0)
        self.brake = self._clamp(brake, 0.0, 1.0)
        self.steering = self._clamp(steering, -1.0, 1.0)

        if requested_gear is not None:
            if isinstance(requested_gear, bool) or not isinstance(requested_gear, int):
                raise TypeError("requested_gear must be an integer or None.")

        self.requested_gear = requested_gear


    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        value = float(value)
        return max(minimum, min(maximum, value))
