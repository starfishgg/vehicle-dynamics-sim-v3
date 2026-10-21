
import pygame

from visualisation.vehicle_renderer import VehicleRenderer
from vehicle.vehicle import Vehicle




def main() -> None:
    pygame.init()

    screen = pygame.display.set_mode((1000, 700))
    pygame.display.set_caption("Driving Simulator - Visualisation Test")
    clock = pygame.time.Clock()

    # Temporary placeholder: we'll replace this with your assembled
    # vehicle once the renderer is displaying correctly.
    vehicle = Vehicle(
        # Supply the same required constructor arguments used by
        # your existing vehicle integration-test fixture.
    )

    renderer = VehicleRenderer(vehicle)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((55, 60, 65))
        renderer.draw(screen, (500, 350))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()




if __name__ == "__main__":
    main()
