
"""Convert world coordinates into screen coordinates."""

from vehicle.vehicle import Vehicle




class Camera:

    def __init__(
            self,
            pixels_per_metre: float = 20.0,
    ) -> None:
        self.pixels_per_metre = pixels_per_metre
        self.x = 0.0
        self.y = 0.0


    def follow(self, vehicle: Vehicle) -> None:
        self.x = vehicle.state.position_x
        self.y = vehicle.state.position_y


    def world_to_screen(
            self,
            world_x: float,
            world_y: float,
            screen_width: int,
            screen_height: int,
    ) -> tuple[int, int]:
        screen_x = screen_width / 2 + (
            world_x - self.x
        ) * self.pixels_per_metre

        # Screen Y increases downwards; world Y increases upwards.
        screen_y = screen_height / 2 - (
            world_y - self.y
        ) * self.pixels_per_metre

        return round(screen_x), round(screen_y)