"""Fixed-timestep physics driver with no rendering dependencies."""


class Simulation:
    """Advances world state at a constant physics rate."""

    def __init__(
        self,
        world,
        physics_dt: float = 0.01,
        duration: float | None = None,
        max_steps: int | None = None,
    ):
        self.world = world
        self.physics_dt = physics_dt
        self.duration = duration
        self.max_steps = max_steps

        self.time = 0.0
        self._steps = 0

    @property
    def running(self) -> bool:
        if self.duration is not None and self.time >= self.duration:
            return False
        if self.max_steps is not None and self._steps >= self.max_steps:
            return False
        return True

    def step(self) -> None:
        """Advance the world by one fixed physics timestep."""
        self.world.step(self.physics_dt)
        self.time += self.physics_dt
        self._steps += 1


def run_simulation(
    world,
    recorder,
    *,
    physics_dt: float,
    duration: float | None = None,
    max_steps: int | None = None,
) -> None:
    """Run physics to completion while sampling snapshots through *recorder*."""
    simulation = Simulation(
        world,
        physics_dt=physics_dt,
        duration=duration,
        max_steps=max_steps,
    )

    recorder.record(world, simulation.time)
    while simulation.running:
        simulation.step()
        recorder.maybe_record(world, simulation.time)
