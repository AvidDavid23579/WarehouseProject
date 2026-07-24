"""Interactive replay viewer with keyboard controls, timeline scrubbing, and HUD."""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from rendering.renderer import Renderer
from simulator.playback import Playback
from simulator.recorder import Recording


class ReplayView:
    """Playback-driven visualization with interactive replay controls."""

    REFRESH_INTERVAL_MS = 33

    KEY_BINDINGS = {
        " ": "toggle_pause",
        "left": "previous_frame",
        "right": "next_frame",
        "r": "restart",
        "home": "restart",
        "end": "skip_to_end",
        "-": "increase_speed",
        "*": "decrease_speed",
    }

    def __init__(self, recording: Recording):
        self.playback = Playback(recording)

        self.renderer = Renderer(recording.static_world)
        self.fig = self.renderer.fig
        self.fig.subplots_adjust(bottom=0.18, top=0.95)
        self.fig.canvas.manager.set_window_title("Warehouse Replay")

        self._slider_sync = False
        self._slider = self._create_slider()
        self._hud = self.fig.text(
            0.01,
            0.97,
            "",
            transform=self.fig.transFigure,
            ha="left",
            va="top",
            fontsize=9,
            family="monospace",
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none", "pad": 4},
        )

        self._animation = None
        self._connect_controls()

    def _create_slider(self) -> Slider:
        slider_ax = self.fig.add_axes([0.12, 0.06, 0.76, 0.03])
        slider = Slider(
            slider_ax,
            "Timeline",
            0.0,
            1.0,
            valinit=self.playback.progress,
            valstep=0.001,
        )
        slider.on_changed(self._on_slider_changed)
        return slider

    def _connect_controls(self) -> None:
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

    def _on_slider_changed(self, value: float) -> None:
        if self._slider_sync:
            return
        self.playback.jump_to_percentage(value)
        self._refresh()

    def _sync_slider(self) -> None:
        self._slider_sync = True
        self.slider.set_val(self.playback.progress)
        self._slider_sync = False

    @property
    def slider(self) -> Slider:
        return self._slider

    def _on_key_press(self, event) -> None:
        action = self.KEY_BINDINGS.get(event.key)
        if action is None:
            return

        if action in {"previous_frame", "next_frame"}:
            self.playback.pause()

        getattr(self.playback, action)()
        self._refresh()

    def _update_hud(self) -> None:
        state = "PAUSED" if self.playback.is_paused else "PLAYING"
        self._hud.set_text(
            f"time: {self.playback.time:6.2f}s / {self.playback.duration:6.2f}s\n"
            f"frame: {self.playback.current_frame:4d} / {self.playback.frame_count:<4d}\n"
            f"speed: {self.playback.speed:4.2f}x\n"
            f"state: {state}"
        )

    def _refresh(self) -> None:
        self.renderer.render_playback(self.playback)
        self._update_hud()
        self._sync_slider()
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        """Open the interactive replay window."""
        self._refresh()

        def on_frame(_frame: int):
            self.playback.advance(self.REFRESH_INTERVAL_MS / 1000.0)
            self._refresh()
            return self.renderer.get_artists()

        self._animation = FuncAnimation(
            self.fig,
            on_frame,
            interval=self.REFRESH_INTERVAL_MS,
            blit=False,
            cache_frame_data=False,
        )

        self.fig.canvas.manager.window.showMaximized()
        plt.show()
