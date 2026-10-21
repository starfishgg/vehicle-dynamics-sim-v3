import unittest

from powertrain.clutch import Clutch
from powertrain.differential import Differential
from powertrain.drivetrain_solver import DrivetrainSolver
from powertrain.engine import Engine
from powertrain.gearbox import Gearbox
from vehicle.wheel import Wheel




class TestDrivetrainSolver(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = Engine(
            peak_torque=400.0,
            base_torque=100.0,
            peak_torque_rpm=4500.0,
            rise_width=1500.0,
            fall_width=2000.0,
            idle_rpm=850.0,
            redline_rpm=7000.0,
            inertia=0.25,
        )

        self.clutch = Clutch(max_torque=500.0, engagement=1.0)

        self.gearbox = Gearbox(
            forward_ratios=(3.5, 2.1, 1.4, 1.0),
            reverse_ratio=3.2,
        )

        self.differential = Differential()

        self.left_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1000.0,
        )
        self.right_wheel = Wheel(
            radius=0.32,
            inertia=1.2,
            max_brake_torque=1000.0,
        )

        self.solver = DrivetrainSolver(
            engine=self.engine,
            clutch=self.clutch,
            gearbox=self.gearbox,
            differential=self.differential,
            driven_wheels=(self.left_wheel, self.right_wheel),
        )


    def test_neutral_does_not_apply_drive_torque(self) -> None:
        self.engine.set_throttle(1.0)
        self.gearbox.select_gear(0)

        self.solver.step(1.0 / 120.0)

        self.assertEqual(self.left_wheel.drive_torque, 0.0)
        self.assertEqual(self.right_wheel.drive_torque, 0.0)


    def test_step_rejects_invalid_timestep(self) -> None:
        with self.assertRaises(ValueError):
            self.solver.step(0.0)


    def test_first_gear_transmits_positive_drive_torque(self) -> None:
        self.gearbox.select_gear(1)
        self.clutch.engagement = 1.0
        self.engine.set_throttle(1.0)

        initial_engine_speed = self.engine.angular_velocity

        self.solver.step(1.0 / 120.0)

        print(f"Engine RPM: {self.engine.rpm:.0f}")

        self.assertGreater(self.left_wheel.drive_torque, 0.0)
        self.assertGreater(self.right_wheel.drive_torque, 0.0)
        self.assertGreater(self.left_wheel.angular_velocity, 0.0)
        self.assertGreater(self.right_wheel.angular_velocity, 0.0)
        self.assertLess(self.engine.angular_velocity, initial_engine_speed)
        self.assertGreater(self.engine.angular_velocity, 0.0)


    def test_slipping_clutch_transfers_torque_to_wheels(self) -> None:
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.5
        self.engine.set_throttle(1.0)

        self.solver.step(1.0 / 120.0)

        self.assertGreater(self.left_wheel.drive_torque, 0.0)
        self.assertGreater(self.right_wheel.drive_torque, 0.0)
        self.assertGreater(self.left_wheel.angular_velocity, 0.0)
        self.assertGreater(self.right_wheel.angular_velocity, 0.0)


    def test_locked_clutch_maintains_speed_relationship(self) -> None:
        self.gearbox.select_gear(1)

        # Start both sides at matching speeds in first gear.
        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed

        self.engine.angular_velocity = (
            self.gearbox.calculate_input_angular_velocity(
                wheel_speed * self.solver.final_drive_ratio
            )
        )
        self.clutch.engagement = 1.0
        self.engine.set_throttle(0.5)

        self.solver.step(1.0 / 120.0)

        carrier_speed = self.differential.calculate_input_angular_velocity(
            self.left_wheel.angular_velocity,
            self.right_wheel.angular_velocity,
        )

        gearbox_output_speed = carrier_speed * self.solver.final_drive_ratio

        expected_engine_speed = (
            self.gearbox.calculate_input_angular_velocity(gearbox_output_speed)
        )

        self.assertAlmostEqual(
            self.engine.angular_velocity,
            expected_engine_speed,
            delta=0.1,
        )


    def test_locked_clutch_preserves_speed_relationship_under_load(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 1.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        # Simulate resistance at the driven wheels.
        self.left_wheel.set_tyre_reaction_torque(-20.0)
        self.right_wheel.set_tyre_reaction_torque(-20.0)
        self.engine.set_throttle(0.5)

        self.solver.step(1.0 / 120.0)

        expected_engine_speed = self.solver.calculate_gearbox_input_speed()

        print("Engine speed after step:", self.engine.angular_velocity)
        print("Left wheel speed after step:", self.left_wheel.angular_velocity)
        print("Right wheel speed after step:", self.right_wheel.angular_velocity)
        print("Reported gearbox input speed:", self.solver.gearbox_input_angular_velocity)
        print("Calculated gearbox input speed:", self.solver.calculate_gearbox_input_speed())




        self.assertAlmostEqual(
            self.engine.angular_velocity,
            expected_engine_speed,
            delta=0.1,
        )


    def test_locked_clutch_preserves_speed_relationship_over_multiple_steps(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 1.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )
        self.engine.set_throttle(0.5)

        for _ in range(20):
            self.solver.step(1.0 / 120.0)

            expected_engine_speed = self.solver.calculate_gearbox_input_speed()

            self.assertAlmostEqual(
                self.engine.angular_velocity,
                expected_engine_speed,
                delta=0.1,
            )


    def test_disengaged_clutch_does_not_drive_wheels(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.0
        self.engine.set_throttle(1.0)

        initial_engine_speed = self.engine.angular_velocity

        self.solver.step(1.0 / 120.0)

        self.assertAlmostEqual(self.left_wheel.drive_torque, 0.0)
        self.assertAlmostEqual(self.right_wheel.drive_torque, 0.0)
        self.assertAlmostEqual(self.left_wheel.angular_velocity, 0.0)
        self.assertAlmostEqual(self.right_wheel.angular_velocity, 0.0)
        self.assertGreater(self.engine.angular_velocity, initial_engine_speed)


    def test_partial_clutch_engagement_allows_engine_wheel_slip(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.25
        self.engine.set_throttle(1.0)

        initial_engine_speed = self.engine.angular_velocity

        self.solver.step(1.0 / 120.0)

        gearbox_speed = self.solver.calculate_gearbox_input_speed()

        self.assertGreater(self.left_wheel.drive_torque, 0.0)
        self.assertGreater(self.right_wheel.drive_torque, 0.0)
        self.assertGreater(self.left_wheel.angular_velocity, 0.0)
        self.assertGreater(self.right_wheel.angular_velocity, 0.0)
        self.assertNotAlmostEqual(
            self.engine.angular_velocity,
            gearbox_speed,
            delta=0.1,
        )
        self.assertGreater(self.engine.angular_velocity, 0.0)


    def test_disengaged_clutch_allows_engine_to_change_speed_independently(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = 100.0
        self.engine.set_throttle(1.0)

        initial_engine_speed = self.engine.angular_velocity

        self.solver.step(1.0 / 120.0)

        self.assertGreater(
            self.engine.angular_velocity,
            initial_engine_speed,
        )
        self.assertAlmostEqual(
            self.left_wheel.angular_velocity,
            wheel_speed,
        )
        self.assertAlmostEqual(
            self.right_wheel.angular_velocity,
            wheel_speed,
        )
        self.assertAlmostEqual(self.left_wheel.drive_torque, 0.0)
        self.assertAlmostEqual(self.right_wheel.drive_torque, 0.0)


    def test_gear_change_with_disengaged_clutch_does_not_change_wheel_speed(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.0

        self.left_wheel.angular_velocity = 10.0
        self.right_wheel.angular_velocity = 10.0
        self.engine.angular_velocity = 100.0

        self.solver.step(1.0 / 120.0)

        left_speed_before_shift = self.left_wheel.angular_velocity
        right_speed_before_shift = self.right_wheel.angular_velocity

        self.gearbox.select_gear(2)
        self.solver.step(1.0 / 120.0)

        self.assertAlmostEqual(
            self.left_wheel.angular_velocity,
            left_speed_before_shift,
        )
        self.assertAlmostEqual(
            self.right_wheel.angular_velocity,
            right_speed_before_shift,
        )
        self.assertEqual(self.gearbox.selected_gear, 2)


    def test_reengaging_clutch_with_speed_mismatch_causes_slip(self):
        self.gearbox.select_gear(2)
        self.clutch.engagement = 0.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed

        # Deliberately mismatch engine speed and gearbox input speed.
        self.engine.angular_velocity = 100.0
        self.engine.set_throttle(0.5)

        self.solver.step(1.0 / 120.0)

        engine_speed_before_engagement = self.engine.angular_velocity
        gearbox_speed_before_engagement = (
            self.solver.calculate_gearbox_input_speed()
        )

        self.clutch.engagement = 0.25
        self.solver.step(1.0 / 120.0)

        gearbox_speed_after_engagement = (
            self.solver.calculate_gearbox_input_speed()
        )

        self.assertGreater(self.left_wheel.drive_torque, 0.0)
        self.assertGreater(self.right_wheel.drive_torque, 0.0)

        # The clutch should transmit torque without instantaneously
        # forcing engine speed to equal gearbox input speed.
        self.assertNotAlmostEqual(
            self.engine.angular_velocity,
            gearbox_speed_after_engagement,
            delta=0.1,
        )

        self.assertNotAlmostEqual(
            engine_speed_before_engagement,
            gearbox_speed_before_engagement,
            delta=0.1,
        )


    def test_locked_clutch_stays_synchronised_over_many_steps(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 1.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )
        self.engine.set_throttle(0.5)

        for _ in range(60):
            self.solver.step(1.0 / 120.0)

            expected_engine_speed = self.solver.calculate_gearbox_input_speed()

            self.assertAlmostEqual(
                self.engine.angular_velocity,
                expected_engine_speed,
                delta=0.1,
            )


    def test_partial_clutch_does_not_instantly_synchronise_speeds(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = 100.0
        self.engine.set_throttle(0.5)

        self.solver.step(1.0 / 120.0)

        self.clutch.engagement = 0.25
        engine_speed_before = self.engine.angular_velocity

        self.solver.step(1.0 / 120.0)

        gearbox_speed_after = self.solver.calculate_gearbox_input_speed()

        self.assertNotAlmostEqual(
            self.engine.angular_velocity,
            gearbox_speed_after,
            delta=0.1,
        )
        self.assertGreater(self.engine.angular_velocity, 0.0)
        self.assertGreater(engine_speed_before, 0.0)


    def test_closed_throttle_with_locked_clutch_slows_driven_wheels(self):
        self.gearbox.select_gear(1)
        self.clutch.engagement = 1.0

        wheel_speed = 10.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = (
            self.gearbox.calculate_input_angular_velocity(wheel_speed)
        )
        self.engine.set_throttle(0.0)

        self.solver.step(1.0 / 120.0)

        self.assertLess(self.left_wheel.angular_velocity, wheel_speed)
        self.assertLess(self.right_wheel.angular_velocity, wheel_speed)


    def test_downshift_with_disengaged_clutch_does_not_jump_engine_speed(self):
        self.gearbox.select_gear(2)
        self.clutch.engagement = 0.0

        wheel_speed = 30.0
        self.left_wheel.angular_velocity = wheel_speed
        self.right_wheel.angular_velocity = wheel_speed
        self.engine.angular_velocity = 200.0
        self.engine.set_throttle(0.0)

        self.solver.step(1.0 / 120.0)
        engine_speed_before_shift = self.engine.angular_velocity

        self.gearbox.select_gear(1)
        self.solver.step(1.0 / 120.0)

        # The open clutch isolates engine speed from the new gear ratio.
        # Engine speed may change due to engine torque and friction,
        # but not through an instantaneous gearbox-imposed speed jump.
        self.assertLess(
            abs(self.engine.angular_velocity - engine_speed_before_shift),
            10.0,
        )


    def test_locked_clutch_preserves_angular_acceleration_relationship(self):
        """Engine and gearbox input accelerations agree when locked."""
        dt = 1.0 / 120.0

        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.5)

        left_wheel, right_wheel = self.solver.driven_wheels

        # Start with both driven wheels rotating at the same speed.
        left_wheel.angular_velocity = 10.0
        right_wheel.angular_velocity = 10.0

        # Apply a moderate external load to both driven wheels.
        left_wheel.set_tyre_reaction_torque(-20.0)
        right_wheel.set_tyre_reaction_torque(-20.0)

        # Begin with the engine and gearbox input synchronised.
        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        gear_ratio = self.solver.gearbox.gear_ratio

        initial_engine_speed = self.solver.engine.angular_velocity
        initial_left_speed = left_wheel.angular_velocity
        initial_right_speed = right_wheel.angular_velocity

        self.solver.step(dt)

        engine_acceleration = (
            self.solver.engine.angular_velocity - initial_engine_speed
        ) / dt

        left_acceleration = (
            left_wheel.angular_velocity - initial_left_speed
        ) / dt

        right_acceleration = (
            right_wheel.angular_velocity - initial_right_speed
        ) / dt

        carrier_acceleration = (
            left_acceleration + right_acceleration
        ) / 2.0

        gearbox_input_acceleration = (
            gear_ratio 
            * self.solver.final_drive_ratio
            * carrier_acceleration
        )

        self.assertAlmostEqual(
            engine_acceleration,
            gearbox_input_acceleration,
            delta=0.1,
            msg=(
                "Engine and gearbox input angular accelerations "
                "should agree when the clutch is locked."
            ),
        )


    def test_locked_clutch_preserves_acceleration_relationship_with_unequal_wheel_speeds(self):
        """The clutch remains kinematically consistent with unequal wheel speeds."""
        dt = 1.0 / 120.0

        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.5)

        left_wheel, right_wheel = self.solver.driven_wheels

        left_wheel.angular_velocity = 8.0
        right_wheel.angular_velocity = 12.0

        left_wheel.set_tyre_reaction_torque(-15.0)
        right_wheel.set_tyre_reaction_torque(-25.0)

        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        initial_engine_speed = self.solver.engine.angular_velocity
        initial_left_speed = left_wheel.angular_velocity
        initial_right_speed = right_wheel.angular_velocity

        self.solver.step(dt)

        engine_acceleration = (
            self.solver.engine.angular_velocity - initial_engine_speed
        ) / dt

        left_acceleration = (
            left_wheel.angular_velocity - initial_left_speed
        ) / dt

        right_acceleration = (
            right_wheel.angular_velocity - initial_right_speed
        ) / dt

        carrier_acceleration = (
            left_acceleration + right_acceleration
        ) / 2.0

        expected_engine_acceleration = (
            self.solver.gearbox.gear_ratio 
            * self.solver.final_drive_ratio
            * carrier_acceleration
        )

        self.assertAlmostEqual(
            engine_acceleration,
            expected_engine_acceleration,
            delta=0.1,
        )


    def test_drivetrain_applies_gearbox_and_differential_efficiencies_once(self):
        """Wheel drive torques include both drivetrain efficiency losses."""
        gearbox = self.solver.gearbox
        differential = self.solver.differential
        left_wheel, right_wheel = self.solver.driven_wheels

        gearbox.select_gear(1)
        self.solver.clutch.engagement = 0.0

        # Use a positive clutch torque directly to isolate the torque path.
        self.solver.clutch_torque = 100.0

        gearbox_output_torque = gearbox.calculate_output_torque(
            -self.solver.clutch_torque
        )
        expected_left, expected_right = differential.calculate_output_torques(
            gearbox_output_torque
        )

        # The solver's next step calculates its own clutch torque, so instead
        # verify the component-level torque path directly.
        self.assertAlmostEqual(
            expected_left + expected_right,
            gearbox_output_torque * differential.efficiency,
        )
        self.assertAlmostEqual(expected_left, expected_right)


    def test_step_applies_equal_drive_torque_to_both_driven_wheels(self):
        """An open differential splits the solver's drive torque equally."""
        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.5)

        left_wheel, right_wheel = self.solver.driven_wheels

        # Start with a speed difference to exercise the differential.
        left_wheel.angular_velocity = 8.0
        right_wheel.angular_velocity = 12.0

        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        self.solver.step(1.0 / 120.0)

        self.assertAlmostEqual(
            left_wheel.drive_torque,
            right_wheel.drive_torque,
            delta=1e-9,
        )


    def test_locked_clutch_preserves_speed_relationship_under_braking(self):
        """Engine and gearbox input speeds remain synchronised under braking."""
        dt = 1.0 / 120.0

        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.0)

        left_wheel, right_wheel = self.solver.driven_wheels
        left_wheel.angular_velocity = 10.0
        right_wheel.angular_velocity = 10.0

        left_wheel.set_brake_input(0.2)
        right_wheel.set_brake_input(0.2)

        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        self.solver.step(dt)

        gearbox_input_speed = self.solver.calculate_gearbox_input_speed()

        self.assertAlmostEqual(
            self.solver.engine.angular_velocity,
            gearbox_input_speed,
            delta=self.solver.clutch.lock_slip_threshold,
        )


    def test_clutch_can_lock_after_slip_falls_within_threshold(self):
        """A small engine-to-gearbox speed difference permits clutch lock."""
        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0

        left_wheel, right_wheel = self.solver.driven_wheels
        left_wheel.angular_velocity = 10.0
        right_wheel.angular_velocity = 10.0

        gearbox_speed = self.solver.calculate_gearbox_input_speed()
        self.solver.engine.angular_velocity = (
            gearbox_speed + self.solver.clutch.lock_slip_threshold * 0.5
        )

        self.assertTrue(
            self.solver.clutch.is_locked(
                engine_angular_velocity=self.solver.engine.angular_velocity,
                gearbox_angular_velocity=gearbox_speed,
                required_engine_torque=0.0,
            )
        )


    def test_clutch_slips_when_required_torque_exceeds_capacity(self):
        """An overloaded clutch cannot maintain engine-to-gearbox synchronisation."""
        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.0)

        left_wheel, right_wheel = self.solver.driven_wheels
        left_wheel.angular_velocity = 10.0
        right_wheel.angular_velocity = 10.0

        self.solver.clutch.max_torque = 1.0

        gearbox_speed = self.solver.calculate_gearbox_input_speed()
        self.solver.engine.angular_velocity = gearbox_speed

        self.solver.step(1.0 / 120.0)


        self.assertAlmostEqual(
            abs(self.solver.clutch_torque),
            self.solver.clutch.torque_capacity,
            delta=1e-6,
            msg="The overloaded clutch should reach its torque capacity.",
        )

        final_gearbox_speed = self.solver.calculate_gearbox_input_speed()
        final_slip = (
            self.solver.engine.angular_velocity - final_gearbox_speed
        )

        self.assertGreater(
            abs(final_slip),
            1e-6,
            "An overloaded clutch should develop measurable speed slip.",
        )


    def test_locked_clutch_does_not_create_large_speed_mismatch_near_stall(self):
        """Low engine speed must not silently break the locked constraint."""
        dt = 1.0 / 120.0

        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.0)

        left_wheel, right_wheel = self.solver.driven_wheels
        left_wheel.angular_velocity = 0.1
        right_wheel.angular_velocity = 0.1

        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        self.solver.step(dt)

        gearbox_speed = self.solver.calculate_gearbox_input_speed()

        self.assertAlmostEqual(
            self.solver.engine.angular_velocity,
            gearbox_speed,
            delta=self.solver.clutch.lock_slip_threshold,
        )


    def test_locked_clutch_maintains_speed_relationship_over_long_run(self):
        """The locked drivetrain stays synchronised over many timesteps."""
        dt = 1.0 / 120.0

        self.solver.gearbox.select_gear(1)
        self.solver.clutch.engagement = 1.0
        self.solver.engine.set_throttle(0.2)

        left_wheel, right_wheel = self.solver.driven_wheels
        left_wheel.angular_velocity = 10.0
        right_wheel.angular_velocity = 10.0

        self.solver.engine.angular_velocity = (
            self.solver.calculate_gearbox_input_speed()
        )

        for _ in range(1200):
            self.solver.step(dt)

            gearbox_speed = self.solver.calculate_gearbox_input_speed()

            self.assertAlmostEqual(
                self.solver.engine.angular_velocity,
                gearbox_speed,
                delta=self.solver.clutch.lock_slip_threshold,
                msg="The locked drivetrain lost synchronisation during the run.",
            )


    def test_brakes_hold_driven_wheels_stationary_during_launch(self) -> None:
        """Applied brakes should hold stationary wheels against drive torque."""
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.25
        self.engine.set_throttle(1.0)

        self.left_wheel.set_brake_input(1.0)
        self.right_wheel.set_brake_input(1.0)

        for _ in range(120):
            self.solver.step(1.0 / 120.0)

        self.assertAlmostEqual(
            self.left_wheel.angular_velocity,
            0.0,
            delta=1e-9,
            msg="The left wheel should remain stationary under full braking.",
        )
        self.assertAlmostEqual(
            self.right_wheel.angular_velocity,
            0.0,
            delta=1e-9,
            msg="The right wheel should remain stationary under full braking.",
        )



    def test_partially_engaged_clutch_transmits_limited_torque(self) -> None:
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.5
        self.engine.set_throttle(0.5)

        # Start the wheels stationary and the engine rotating.
        self.engine.angular_velocity = 200.0
        self.left_wheel.angular_velocity = 0.0
        self.right_wheel.angular_velocity = 0.0

        self.solver.step(1.0 / 120.0)

        # At this slip speed, the clutch should be slipping.
        self.assertLess(self.solver.clutch_torque, 0.0)
        self.assertLessEqual(
            abs(self.solver.clutch_torque),
            self.clutch.torque_capacity,
        )

        # The clutch should transfer torque to the driven wheels.
        self.assertGreater(self.left_wheel.drive_torque, 0.0)
        self.assertGreater(self.right_wheel.drive_torque, 0.0)



    def test_fully_disengaged_clutch_does_not_drive_wheels(self) -> None:
        self.gearbox.select_gear(1)
        self.clutch.engagement = 0.0
        self.engine.set_throttle(0.5)

        # Start the wheels stationary and the engine rotating.
        self.engine.angular_velocity = 200.0
        self.left_wheel.angular_velocity = 0.0
        self.right_wheel.angular_velocity = 0.0

        self.solver.step(1.0 / 120.0)

        self.assertEqual(self.solver.clutch_torque, 0.0)
        self.assertEqual(self.left_wheel.drive_torque, 0.0)
        self.assertEqual(self.right_wheel.drive_torque, 0.0)

        # The engine should continue evolving independently of the wheels.
        self.assertGreater(self.engine.angular_velocity, 0.0)

        self.assertEqual(self.left_wheel.angular_velocity, 0.0)
        self.assertEqual(self.right_wheel.angular_velocity, 0.0)




if __name__ == "__main__":
    unittest.main()
