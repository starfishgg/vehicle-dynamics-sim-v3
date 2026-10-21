from configs.tyre_presets import create_road_tyre
from core.physics_constants import GRAVITY

tyre = create_road_tyre()
normal_load = 1500.0

for slip_ratio in (0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0):
    force = tyre.calculate_longitudinal_force(
        slip_ratio,
        normal_load,
    )
    print(f"Slip: {slip_ratio:5.2f} | Force: {force:8.1f} N")


    