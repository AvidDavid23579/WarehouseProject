"""Export recorded simulations to video without rerunning physics."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

from rendering.renderer import Renderer
from simulator.playback import Playback
from simulator.recorder import Recording


def export_video(
    recording: Recording,
    output_path: str | Path,
    *,
    fps: float | None = None,
    width: int = 1920,
    height: int = 1080,
    dpi: int = 100,
) -> None:
    """Render a recording to MP4 via Playback → Renderer → file."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    playback = Playback(recording)
    fig = plt.figure(
        figsize=(width / dpi, height / dpi),
        dpi=dpi,
    )

    ax = fig.add_subplot(111)
    renderer = Renderer(recording.static_world, fig=fig, ax=ax)
    export_fps = fps or recording.recording_fps

    writer = FFMpegWriter(fps=export_fps)
    with writer.saving(renderer.fig, str(output_path), dpi=dpi):
        for _ in range(playback.frame_count):
            renderer.render_playback(playback)
            writer.grab_frame()
            if not playback.is_finished():
                playback.next_frame()

    plt.close(renderer.fig)
