
from core.physics_state import PhysicsState
from powertrain.drivetrain_solver import DrivetrainSolver
from vehicle.wheel import Wheel
from tyres.tyre import Tyre
from vehicle.chassis import Chassis




class Vehicle:
    """Assembles the components that make up a simulated vehicle."""

    def __init__(
            self,
            *,
            chassis: Chassis,
            front_left_wheel: Wheel,
            front_right_wheel: Wheel,
            rear_left_wheel: Wheel,
            rear_right_wheel: Wheel,
            front_left_tyre: Tyre,
            front_right_tyre: Tyre,
            rear_left_tyre: Tyre,
            rear_right_tyre: Tyre,
            drivetrain: DrivetrainSolver,
            state: PhysicsState | None = None,
    ) -> None:

        self.chassis = chassis

        self.front_left_wheel = front_left_wheel
        self.front_right_wheel = front_right_wheel
        self.rear_left_wheel = rear_left_wheel
        self.rear_right_wheel = rear_right_wheel

        self.front_left_tyre = front_left_tyre
        self.front_right_tyre = front_right_tyre
        self.rear_left_tyre = rear_left_tyre
        self.rear_right_tyre = rear_right_tyre

        self.drivetrain = drivetrain
        self.state = state if state is not None else PhysicsState()


    @property
    def wheels(self) -> tuple[Wheel, Wheel, Wheel, Wheel]:
        "Returns wheels in front-left, front-right, rear-left, rear-right order."
        return (
            self.front_left_wheel,
            self.front_right_wheel,
            self.rear_left_wheel,
            self.rear_right_wheel,
        )


    @property
    def tyres(self) -> tuple[Tyre, Tyre, Tyre, Tyre]:
        """Returns tyres in front-left, front-right, rear-left, rear-right order."""
        return (
            self.front_left_tyre,
            self.front_right_tyre,
            self.rear_left_tyre,
            self.rear_right_tyre,
        )

    @property
    def mass(self) -> float:
        """Return the current total mass, including driver and fuel."""
        return self.chassis.total_mass



