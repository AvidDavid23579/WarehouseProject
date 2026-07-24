"""Fixed-timestep physics loop driven by matplotlib animation callbacks."""

import time


class Simulation:
    """Runs world physics at a fixed rate, decoupled from render frame rate."""

    def __init__(
        self,
        world,
        renderer,
        physics_dt: float = 0.01,
        start_delay: float = 3.0,
        max_steps_per_frame: int = 50,
    ):
        self.world = world
        self.renderer = renderer
        self.physics_dt = physics_dt
        self.start_delay = start_delay
        # Prevents a long UI stall from replaying minutes of catch-up physics.
        self.max_steps_per_frame = max_steps_per_frame

        self._accumulator = 0.0
        self._running = False
        self._start_time = time.perf_counter()
        self._last_time = self._start_time

    def on_frame(self, _frame: int):
        """Matplotlib FuncAnimation callback — step physics and redraw."""
        now = time.perf_counter()

        if not self._running:
            if now - self._start_time < self.start_delay:
                return self.renderer.get_artists()
            self._running = True
            self._last_time = now
            return self.renderer.get_artists()

        elapsed = now - self._last_time
        self._last_time = now
        self._accumulator += elapsed

        steps = 0
        while self._accumulator >= self.physics_dt and steps < self.max_steps_per_frame:
            self.world.step(self.physics_dt)
            self._accumulator -= self.physics_dt
            steps += 1

        return self.renderer.draw(self.world.snapshot())
