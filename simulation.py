import time


class Simulation:
    def __init__(self, world, renderer, physics_dt=0.01, start_delay=3.0, max_steps_per_frame=50):
        self.world = world
        self.renderer = renderer
        self.physics_dt = physics_dt  # Physics run at a constant 100 Hz
        self.start_delay = start_delay
        # Caps catch-up steps after a stall (e.g. window drag) so the sim
        # doesn't try to replay minutes of physics in one frame.
        self.max_steps_per_frame = max_steps_per_frame

        self._accumulator = 0.0
        self._running = False
        self._start_time = time.perf_counter()
        self._last_time = self._start_time

    def on_frame(self, frame):
        now = time.perf_counter()

        if not self._running:
            if now - self._start_time < self.start_delay:
                return self.renderer.get_artists()
            self._running = True
            self._last_time = now
            return self.renderer.get_artists()

        # Pauses the simulator at the start to observe initial state
        elapsed = now - self._last_time
        self._last_time = now
        self._accumulator += elapsed

        steps = 0

        # Start the simulation with frame freeze condition
        while self._accumulator >= self.physics_dt and steps < self.max_steps_per_frame:
            self.world.step(self.physics_dt)
            self._accumulator -= self.physics_dt
            steps += 1

        return self.renderer.draw(self.world.snapshot())
