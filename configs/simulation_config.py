"""Configuration shared by the vehicle simulation."""




class SimulationConfig:
    """
    Numerical settings for the physics simulation.
    
    Physics calculations use SI units. The simulation advencas in 
    fixed-size steps; rendering and wall-clock timing are separate.
    """

    def __init__(
            self,
            fixed_timestep: float = 1.0 / 120.0,
            gravity: float = 9.81,
    ):
        if fixed_timestep <= 0.0:
            raise ValueError("fixed_timestep must be positive.")

        if gravity <= 0.0:
            raise ValueError("gravity must be positive.")

        self.fixed_timestep = float(fixed_timestep)
        self.gravity = float(gravity)

