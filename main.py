
"""Run the vehicle physics and Pygame visualisation."""

import pygame

from vehicle.vehicle_factory import create_rear_wheel_drive_car
from visualisation.camera import Camera
from visualisation.vehicle_renderer import VehicleRenderer
from core.driver_input import DriverInput

SCREEN_WIDTH = 1100
SCREEN_HEIGHT = 750
PHYSICS_TIMESTEP = 1.0 / 120.0
MAX_FRAME_TIME = 0.1




def main() -> None:
    pygame.init()

    screen = pygame.display.set_mode(
        (SCREEN_WIDTH, SCREEN_HEIGHT)
    )
    pygame.display.set_caption("Driving Simulator v3")

    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 26)
    small_font = pygame.font.Font(None, 21)

    vehicle, vehicle_dynamics = create_rear_wheel_drive_car()
    engine = vehicle.drivetrain.engine

    renderer = VehicleRenderer(vehicle)
    camera = Camera(renderer.pixels_per_metre)

    accumulator = 0.0
    running = True

    diagnostic_timer = 0.0

    while running:
        frame_time = min(
            clock.tick(60) / 1000.0,
            MAX_FRAME_TIME,
        )

        requested_gear = None

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_q:
                    current_gear = vehicle.drivetrain.gearbox.selected_gear
                    requested_gear = max(-1, current_gear -1)

                elif event.key == pygame.K_e:
                    current_gear = vehicle.drivetrain.gearbox.selected_gear
                    requested_gear = min(
                        vehicle.drivetrain.gearbox.gear_count,
                        current_gear + 1,
                    )

        keys = pygame.key.get_pressed()



        throttle = 1.0 if keys[pygame.K_w] else 0.0
        brake = 1.0 if keys[pygame.K_s] else 0.0

        steering = 0.0
        if keys[pygame.K_a]:
            steering -= 1.0
        if keys[pygame.K_d]:
            steering += 1.0

        clutch_pedal = 1.0 if keys[pygame.K_LSHIFT] else 0.0

        driver_input = DriverInput(
            throttle=throttle,
            brake=brake,
            steering=steering,
            clutch_pedal=clutch_pedal,
            requested_gear=requested_gear,
        )

        accumulator += frame_time

        while accumulator >= PHYSICS_TIMESTEP:
            vehicle_dynamics.step(PHYSICS_TIMESTEP, driver_input)
            accumulator -= PHYSICS_TIMESTEP

            diagnostic_timer += PHYSICS_TIMESTEP

            if diagnostic_timer >= 0.25:
                drivetrain = vehicle.drivetrain
                clutch = drivetrain.clutch
                rear_left, rear_right = drivetrain.driven_wheels

                clutch_slip = (
                    engine.angular_velocity
                    - drivetrain.gearbox_input_angular_velocity
                )



                gearbox = drivetrain.gearbox

                vehicle_speed_kmh = vehicle.state.speed * 3.6
                engine_rpm = engine.rpm

                rear_left_speed_kmh = (
                    rear_left.linear_speed * 3.6
                )
                rear_right_speed_kmh = (
                    rear_right.linear_speed * 3.6
                )


                """ print(
                    f"\nVehicle: {vehicle_speed_kmh:.1f} km/h | "
                    f"Engine: {engine_rpm:.0f} RPM | "
                    f"Gear: {gearbox.selected_gear} | "
                    f"Gear ratio: {gearbox.gear_ratio} | "
                    f"Clutch engagement: {clutch.engagement:.2f}"
                )

                print(
                    f"Engine angular speed: {engine.angular_velocity:.1f} rad/s | "
                    f"Gearbox input speed: "
                    f"{drivetrain.gearbox_input_angular_velocity:.1f} rad/s | "
                    f"Clutch slip: {clutch_slip:.1f} rad/s"
                )

                print(
                    f"Rear wheel road speeds: "
                    f"left={rear_left_speed_kmh:.1f} km/h, "
                    f"right={rear_right_speed_kmh:.1f} km/h"
                )

                for index, wheel in enumerate(vehicle.wheels):
                    tyre_force = vehicle_dynamics.longitudinal_tyre_forces[index]

                    print(
                        f"Wheel {index}: "
                        f"object_id={id(wheel)}, "
                        f"force={tyre_force:.3f} N, "
                        f"reaction={wheel.tyre_reaction_torque:.3f} Nm, "
                        f"expected={-tyre_force * wheel.radius:.3f} Nm, "
                        f"drive={wheel.drive_torque:.3f} Nm, "
                        f"speed={wheel.angular_velocity:.3f} rad/s"
                    ) """


                diagnostic_timer = 0.0



        camera.follow(vehicle)

        screen.fill((48 , 53, 5,8))

        # World grid: each line is ten metres apart.
        grid_spacing = round(10.0 * camera.pixels_per_metre)

        offset_x = round(
            SCREEN_WIDTH / 2 - camera.x * camera.pixels_per_metre
        ) % grid_spacing
        offset_y = round(
            SCREEN_HEIGHT / 2 + camera.y * camera.pixels_per_metre
        ) % grid_spacing

        for x in range(offset_x, SCREEN_WIDTH, grid_spacing):
            pygame.draw.line(
                screen, (62, 67, 72), (x, 0), (x, SCREEN_HEIGHT)
            )

        for y in range(offset_y, SCREEN_HEIGHT, grid_spacing):
            pygame.draw.line(
                screen, (62, 67, 72), (0, y), (SCREEN_WIDTH, y)
            )

        car_screen_position = camera.world_to_screen(
            vehicle.state.position_x,
            vehicle.state.position_y,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
        )

        renderer.draw(screen, car_screen_position)

        speed_kmh = vehicle.state.speed * 3.6

        telemetry = [
            f"Speed: {speed_kmh:6.1f} km/h",
            f"Engine: {engine.rpm:6.0f} RPM",
            f"Gear: {vehicle.drivetrain.gearbox.selected_gear}",
            f"Throttle: {driver_input.throttle * 100:3.0f}%",
            f"Brake: {driver_input.brake * 100:3.0f}%",
            f"Steering input: {driver_input.steering:+.1f}",
            f"Clutch pedal: {driver_input.clutch_pedal:+.1f}",
            f"Clutch position: {vehicle.drivetrain.clutch.engagement:+.1f}",
            f"Selected gear: {vehicle.drivetrain.gearbox.selected_gear}",
        ]

        panel = pygame.Surface((280, 250), pygame.SRCALPHA)
        panel.fill((15, 18, 22, 220))
        screen.blit(panel, (15, 15))

        for index, line in enumerate(telemetry):
            text_surface = font.render(
                line, True, (235, 240, 245)
            )
            screen.blit(text_surface, (28, 25 + index * 27))

        help_text = small_font.render(
            "W: throttle   S: brake   A/D: steering input   Esc: quit",
            True,
            (235, 240, 245),
        )
        screen.blit(
            help_text,
            (15, SCREEN_HEIGHT - 30),
        )

        pygame.display.flip()

    pygame.quit()




if __name__ == "__main__":
    main()

