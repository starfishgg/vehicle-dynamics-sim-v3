
"""Pygame rendering for the vehicle."""

import math
import pygame

from vehicle.vehicle import Vehicle




class VehicleRenderer:
    def __init__(
        self,
        vehicle: Vehicle,
        pixels_per_metre: float = 20.0,
    ) -> None:
        if not math.isfinite(pixels_per_metre) or pixels_per_metre <= 0:
            raise ValueError("pixels_per_metre must be finite and positive.")

        self.vehicle = vehicle
        self.pixels_per_metre = pixels_per_metre

        self.body_length = round(4.2 * pixels_per_metre)
        self.body_width = round(1.8 * pixels_per_metre)


    def draw(
        self,
        surface: pygame.Surface,
        screen_position: tuple[int, int],
    ) -> None:
        """Draw the car at a screen position, facing its physics heading."""
        length = self.body_length
        width = self.body_width

        car_image = pygame.Surface(
            (length + 8, width + 8),
            pygame.SRCALPHA,
        )

        centre_x = car_image.get_width() // 2
        centre_y = car_image.get_height() // 2

        body = pygame.Rect(4, 4, length, width)

        pygame.draw.rect(
            car_image,
            (40, 125, 220),
            body,
            border_radius=12,
        )

        # Front windscreen and rear window.
        pygame.draw.rect(
            car_image,
            (35, 43, 55),
            pygame.Rect(
                centre_x + 12,
                centre_y - round(width * 0.34),
                round(length * 0.20),
                round(width * 0.68),
            ),
            border_radius=5,
        )

        pygame.draw.rect(
            car_image,
            (35, 43, 55),
            pygame.Rect(
                centre_x - round(length * 0.34),
                centre_y - round(width * 0.34),
                round(length * 0.16),
                round(width * 0.68),
            ),
            border_radius=5,
        )

        # Four wheels, positioned along the sides of the car.
        wheel_length = round(length * 0.19)
        wheel_width = max(5, round(width * 0.19))

        for wheel_x in (
            round(length * 0.23),
            round(length * 0.75),
        ):
            for wheel_y in (
                4 - wheel_width // 2,
                4 + width - wheel_width // 2,
            ):
                pygame.draw.rect(
                    car_image,
                    (20, 20, 22),
                    pygame.Rect(
                        wheel_x,
                        wheel_y,
                        wheel_length,
                        wheel_width,
                    ),
                    border_radius=3,
                )

        # Headlights mark the front (+X) end.
        for light_y in (
            centre_y - round(width * 0.32),
            centre_y + round(width * 0.32),
        ):
            pygame.draw.circle(
                car_image,
                (255, 240, 170),
                (length + 2, light_y),
                max(3, round(self.pixels_per_metre * 0.10)),
            )

        # Physics heading is counter-clockwise in world space.
        # Pygame's positive rotation is counter-clockwise on screen.
        rotated_image = pygame.transform.rotate(
            car_image,
            math.degrees(self.vehicle.state.heading),
        )

        surface.blit(
            rotated_image,
            rotated_image.get_rect(center=screen_position),
        )
