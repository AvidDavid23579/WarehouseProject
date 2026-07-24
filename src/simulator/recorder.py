"""Collects dynamic world snapshots during simulator at a configurable rate."""

import pickle
from dataclasses import dataclass
from pathlib import Path

from simulator.snapshot import SimulationSnapshot, StaticWorld

RECORDING_VERSION = 1


@dataclass(frozen=True, slots=True)
class Recording:
    """Complete snapshot sequence and static scene for replay."""

    snapshots: tuple[SimulationSnapshot, ...]
    recording_fps: float
    physics_dt: float
    static_world: StaticWorld
    version: int = RECORDING_VERSION

    def save(self, path: str | Path) -> None:
        """Persist the recording to disk."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("wb") as file:
            pickle.dump(self, file, protocol=pickle.HIGHEST_PROTOCOL)

    @classmethod
    def load(cls, path: str | Path) -> "Recording":
        """Restore a recording previously saved with :meth:`save`."""
        with Path(path).open("rb") as file:
            recording = pickle.load(file)
        if not isinstance(recording, cls):
            raise TypeError(f"Expected Recording, got {type(recording)!r}")
        return recording

    @property
    def frame_count(self) -> int:
        return len(self.snapshots)

    @property
    def duration(self) -> float:
        if not self.snapshots:
            return 0.0
        return self.snapshots[-1].time


class Recorder:
    """Samples dynamic world state independently of the physics timestep."""

    def __init__(
        self,
        recording_fps: float,
        physics_dt: float,
        static_world: StaticWorld,
        *,
        record_every_step: bool = False,
    ):
        self.recording_fps = recording_fps
        self.physics_dt = physics_dt
        self.static_world = static_world
        self.record_every_step = record_every_step

        self._snapshots: list[SimulationSnapshot] = []
        self._sample_interval = 1.0 / recording_fps
        self._next_sample_time = 0.0

    def record(self, world, time: float) -> None:
        """Capture the current dynamic state."""
        self._snapshots.append(world.capture_snapshot(time))

    def maybe_record(self, world, time: float) -> None:
        """Record only when the configured sampling interval has elapsed."""
        if self.record_every_step:
            self.record(world, time)
            return

        if time + 1e-9 >= self._next_sample_time:
            self.record(world, time)
            self._next_sample_time += self._sample_interval

    def finish(self) -> Recording:
        """Return the completed recording."""
        return Recording(
            snapshots=tuple(self._snapshots),
            recording_fps=self.recording_fps,
            physics_dt=self.physics_dt,
            static_world=self.static_world,
        )

    @property
    def frame_count(self) -> int:
        return len(self._snapshots)
