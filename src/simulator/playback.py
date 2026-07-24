"""Iterates recorded snapshots for visualization and replay controls."""

import bisect

from common.utils import clamp
from simulator.recorder import Recording
from simulator.snapshot import SimulationSnapshot


class Playback:
    """High-level navigation through a recording without simulator access."""

    MIN_SPEED = 0.25
    MAX_SPEED = 4.0
    SPEED_STEP = 0.25

    def __init__(self, recording: Recording):
        self.recording = recording
        self.speed = 1.0
        self.paused = False

        self._frame_index = 0
        self._accumulator = 0.0
        self._snapshot_times = tuple(snapshot.time for snapshot in recording.snapshots)

    # --- Snapshot access -----------------------------------------------------

    @property
    def current(self) -> SimulationSnapshot:
        return self.recording.snapshots[self._frame_index]

    # --- Metadata ------------------------------------------------------------

    @property
    def time(self) -> float:
        return self.current.time

    @property
    def duration(self) -> float:
        return self.recording.duration

    @property
    def frame_count(self) -> int:
        return self.recording.frame_count

    @property
    def current_frame(self) -> int:
        """One-based frame index for display."""
        return self._frame_index + 1

    @property
    def progress(self) -> float:
        if self.frame_count <= 1:
            return 0.0
        return self._frame_index / (self.frame_count - 1)

    @property
    def frame_duration(self) -> float:
        return 1.0 / self.recording.recording_fps

    @property
    def is_paused(self) -> bool:
        return self.paused

    # --- Navigation ----------------------------------------------------------

    def restart(self) -> None:
        """Jump to the first frame."""
        self._seek_to_index(0)

    def skip_to_end(self) -> None:
        """Jump to the final frame."""
        self._seek_to_index(self.frame_count - 1)

    def jump_to_time(self, seconds: float) -> None:
        """Jump to the frame closest to *seconds* of simulator time."""
        if not self._snapshot_times:
            return
        seconds = clamp(seconds, 0.0, self.duration)
        index = bisect.bisect_left(self._snapshot_times, seconds)
        if index >= self.frame_count:
            index = self.frame_count - 1
        elif index > 0:
            before = self._snapshot_times[index - 1]
            after = self._snapshot_times[index]
            if abs(seconds - before) <= abs(after - seconds):
                index -= 1
        self._seek_to_index(index)

    def jump_to_percentage(self, percent: float) -> None:
        """Jump to a normalized position in the recording [0, 1]."""
        if self.frame_count <= 1:
            self._seek_to_index(0)
            return
        percent = clamp(percent, 0.0, 1.0)
        index = round(percent * (self.frame_count - 1))
        self._seek_to_index(int(index))

    def next_frame(self) -> None:
        """Advance by one frame."""
        self._seek_to_index(self._frame_index + 1)

    def previous_frame(self) -> None:
        """Rewind by one frame."""
        self._seek_to_index(self._frame_index - 1)

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def toggle_pause(self) -> None:
        self.paused = not self.paused

    def increase_speed(self) -> None:
        self.speed = min(self.MAX_SPEED, self.speed + self.SPEED_STEP)

    def decrease_speed(self) -> None:
        self.speed = max(self.MIN_SPEED, self.speed - self.SPEED_STEP)

    def is_finished(self) -> bool:
        return self._frame_index >= self.frame_count - 1

    def advance(self, dt: float) -> None:
        """Advance playback by *dt* seconds of real time at the current speed."""
        if self.paused or self.is_finished():
            return

        self._accumulator += dt * self.speed
        while self._accumulator >= self.frame_duration and not self.is_finished():
            self._accumulator -= self.frame_duration
            self._frame_index += 1

    # --- Internal ------------------------------------------------------------

    def _seek_to_index(self, index: int) -> None:
        if self.frame_count == 0:
            return
        self._frame_index = max(0, min(index, self.frame_count - 1))
        self._accumulator = 0.0
