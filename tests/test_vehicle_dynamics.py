
"""Tests for longitudinal vehicle dynamics."""

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from core.physics_constants import AIR_DENSITY, GRAVITY
from vehicle.vehicle_dynamics import VehicleDynamics




class VehicleDynamicsTests(unittest.TestCase):
    """Verify longitudinal forces and simulation-step behaviour."""

    def make_dynamics(
        self,
        *,
        speed: float = 10.0,
        wheel_speeds: tuple[float, ...] = (10.0, 10.0, 10.0, 10.0),
        tyre_forces: tuple[float, ...] = (100.0, 100.0, 100.0, 100.0),
        mass: float = 1000.0,
        drag_coefficient: float = 0.0,
        frontal_area: float = 2.1,
        rolling_resistance_coefficient: float = 0.0,
        surface_grip: float = 1.0,
    ):
        """Build lightweight components for isolated physics tests."""
        wheels = []
        tyres = []

        for wheel_speed, tyre_force in zip(wheel_speeds, tyre_forces):
            wheel = Mock()
            wheel.linear_speed = wheel_speed
            wheel.radius = 0.3
            wheel.set_tyre_reaction_torque = Mock()
            wheel.integrate_angular_velocity = Mock()
            wheels.append(wheel)

            tyre = Mock()
            tyre.calculate_longitudinal_force.return_value = tyre_force
            tyre.calculate_reaction_torque.side_effect = (
                lambda force, radius: -force * radius
            )
            tyres.append(tyre)

        chassis = SimpleNamespace(
            total_mass=mass,
            drag_coefficient=drag_coefficient,
            frontal_area=frontal_area,
            rolling_resistance_coefficient=(
                rolling_resistance_coefficient
            ),
            calculate_static_axle_loads=Mock(
                return_value=(
                    mass * GRAVITY * 0.55,
                    mass * GRAVITY * 0.45,
                )
            ),
            calculate_dynamic_axle_loads=Mock(
                return_value=(5000.0, 5000.0)
            ),
        )

        state = SimpleNamespace(
            longitudinal_velocity=speed,
            lateral_velocity=0.0,
            longitudinal_acceleration=0.0,
            heading=0.0,
            position_x=0.0,
            position_y=0.0,
        )

        drivetrain = Mock()
        drivetrain.driven_wheels = (wheels[2], wheels[3])

        vehicle = SimpleNamespace(
            chassis=chassis,
            wheels=tuple(wheels),
            tyres=tuple(tyres),
            state=state,
            drivetrain=drivetrain,
        )

        dynamics = VehicleDynamics(
            vehicle,
            surface_grip=surface_grip,
        )

        return dynamics, vehicle, wheels, tyres, drivetrain


    def test_slip_ratio_is_positive_when_wheel_is_faster(self):
        slip = VehicleDynamics.calculate_slip_ratio(
            wheel_linear_speed=12.0,
            vehicle_speed=10.0,
        )

        self.assertAlmostEqual(slip, 2.0 / 12.0)


    def test_slip_ratio_is_negative_when_wheel_is_slower(self):
        slip = VehicleDynamics.calculate_slip_ratio(
            wheel_linear_speed=8.0,
            vehicle_speed=10.0,
        )

        self.assertAlmostEqual(slip, -0.2)


    def test_slip_ratio_is_finite_at_standstill(self):
        slip = VehicleDynamics.calculate_slip_ratio(
            wheel_linear_speed=0.0,
            vehicle_speed=0.0,
        )

        self.assertEqual(slip, 0.0)
        self.assertTrue(math.isfinite(slip))


    def test_surface_grip_scales_longitudinal_tyre_force(self):
        dynamics, _, wheels, tyres, _ = self.make_dynamics(
            surface_grip=0.7,
            tyre_forces=(100.0, 100.0, 100.0, 100.0),
        )

        forces = dynamics._calculate_tyre_forces()

        self.assertEqual(forces, (70.0, 70.0, 70.0, 70.0))

        for wheel, tyre in zip(wheels, tyres):
            tyre.calculate_longitudinal_force.assert_called_once()
            wheel.set_tyre_reaction_torque.assert_called_once_with(
                -70.0 * wheel.radius
            )


    def test_zero_surface_grip_produces_zero_tyre_force(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            surface_grip=0.0,
        )

        forces = dynamics._calculate_tyre_forces()

        self.assertEqual(forces, (0.0, 0.0, 0.0, 0.0))


    def test_drag_opposes_forward_motion(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            drag_coefficient=0.3,
            frontal_area=2.1,
        )

        drag = dynamics._calculate_drag_force(20.0)

        expected = (
            -0.5 * AIR_DENSITY * 0.3 * 2.1 * 20.0**2
        )

        self.assertAlmostEqual(drag, expected)
        self.assertLess(drag, 0.0)


    def test_drag_opposes_reverse_motion(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            drag_coefficient=0.3,
            frontal_area=2.1,
        )

        drag = dynamics._calculate_drag_force(-20.0)

        self.assertGreater(drag, 0.0)
        self.assertAlmostEqual(
            drag,
            -dynamics._calculate_drag_force(20.0),
        )


    def test_rolling_resistance_opposes_forward_motion(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            mass=1000.0,
            rolling_resistance_coefficient=0.012,
        )

        resistance = dynamics._calculate_rolling_resistance(10.0)

        self.assertAlmostEqual(
            resistance,
            -0.012 * 1000.0 * GRAVITY,
        )
        self.assertLess(resistance, 0.0)


    def test_rolling_resistance_opposes_reverse_motion(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            rolling_resistance_coefficient=0.012,
        )

        resistance = dynamics._calculate_rolling_resistance(-10.0)

        self.assertGreater(resistance, 0.0)


    def test_rolling_resistance_is_zero_at_rest(self):
        dynamics, _, _, _, _ = self.make_dynamics(
            rolling_resistance_coefficient=0.012,
        )

        self.assertEqual(
            dynamics._calculate_rolling_resistance(0.0),
            0.0,
        )


    def test_step_calculates_acceleration_and_updates_position(self):
        dynamics, vehicle, wheels, _, drivetrain = self.make_dynamics(
            speed=10.0,
            tyre_forces=(100.0, 100.0, 100.0, 100.0),
            mass=1000.0,
        )

        dynamics.step(0.01)

        # Total tyre force is 400 N; road loads are zero.
        self.assertAlmostEqual(
            vehicle.state.longitudinal_acceleration,
            0.4,
        )
        self.assertAlmostEqual(
            vehicle.state.longitudinal_velocity,
            10.004,
        )
        self.assertAlmostEqual(
            vehicle.state.position_x,
            10.004 * 0.01,
        )

        drivetrain.step.assert_called_once_with(0.01)

        # Front wheels are not driven in this test configuration.
        wheels[0].integrate_angular_velocity.assert_called_once_with(0.01)
        wheels[1].integrate_angular_velocity.assert_called_once_with(0.01)

        # Driven rear wheels are integrated by the drivetrain solver.
        wheels[2].integrate_angular_velocity.assert_not_called()
        wheels[3].integrate_angular_velocity.assert_not_called()


    def test_step_includes_aerodynamic_drag_in_acceleration(self):
        dynamics, vehicle, _, _, _ = self.make_dynamics(
            speed=10.0,
            tyre_forces=(0.0, 0.0, 0.0, 0.0),
            mass=1000.0,
            drag_coefficient=0.3,
            frontal_area=2.1,
        )

        dynamics.step(0.01)

        expected_drag = -0.5 * AIR_DENSITY * 0.3 * 2.1 * 10.0**2
        expected_acceleration = expected_drag / 1000.0

        self.assertAlmostEqual(
            vehicle.state.longitudinal_acceleration,
            expected_acceleration,
        )


    def test_invalid_surface_grip_is_rejected(self):
        for grip in (-0.1, float("nan"), float("inf")):
            with self.subTest(grip=grip):
                with self.assertRaises(ValueError):
                    self.make_dynamics(surface_grip=grip)


    def test_invalid_time_step_is_rejected(self):
        dynamics, _, _, _, drivetrain = self.make_dynamics()

        for dt in (0.0, -0.01, float("nan"), float("inf")):
            with self.subTest(dt=dt):
                with self.assertRaises(ValueError):
                    dynamics.step(dt)

        drivetrain.step.assert_not_called()


    def test_all_wheels_receive_their_calculated_tyre_reaction_torque(
        self,
    ):
        tyre_forces = (100.0, -200.0, 300.0, -400.0)

        dynamics, vehicle, wheels, tyres, _ = self.make_dynamics(
            tyre_forces=tyre_forces,
        )

        dynamics.step(0.01)

        for index, (wheel, tyre, force) in enumerate(
            zip(wheels, tyres, tyre_forces)
        ):
            with self.subTest(wheel_index=index):
                expected_torque = -force * wheel.radius

                tyre.calculate_reaction_torque.assert_called_once_with(
                    force,
                    wheel.radius,
                )
                wheel.set_tyre_reaction_torque.assert_called_once_with(
                    expected_torque,
                )

                self.assertAlmostEqual(
                    wheel.set_tyre_reaction_torque.call_args.args[0],
                    expected_torque,
                )



    def test_real_wheel_reaction_torques_match_calculated_tyre_forces(
        self,
    ):
        from vehicle.vehicle_factory import create_rear_wheel_drive_car

        vehicle, dynamics = create_rear_wheel_drive_car()

        # Remove drivetrain torque from the equation for this test.
        vehicle.drivetrain.clutch.engagement = 0.0
        vehicle.drivetrain.engine.set_throttle(0.0)

        # Give all four wheels and the vehicle the same forward speed.
        vehicle.state.longitudinal_velocity = 10.0

        for wheel in vehicle.wheels:
            wheel.angular_velocity = 10.0 / wheel.radius

        dynamics.step(1.0 / 120.0)

        forces = dynamics.longitudinal_tyre_forces

        for index, (wheel, force) in enumerate(
            zip(vehicle.wheels, forces)
        ):
            with self.subTest(wheel_index=index):
                expected_torque = -force * wheel.radius

                self.assertAlmostEqual(
                    wheel.tyre_reaction_torque,
                    expected_torque,
                    places=7,
                    msg=(
                        f"Wheel {index}: force={force:.6f} N, "
                        f"stored torque={wheel.tyre_reaction_torque:.6f} Nm, "
                        f"expected torque={expected_torque:.6f} Nm"
                    ),
                )





if __name__ == "__main__":
    unittest.main()
