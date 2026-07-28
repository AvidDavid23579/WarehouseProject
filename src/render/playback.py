import time

import pygame

from config import PHYSICS_DT
from entities.robot import RobotFrame
from render.renderer import Renderer
from simulator.world import WorldFrame


class Playback:
    FPS = 60

    def __init__(self, frames: list[WorldFrame]):
        self.frames = frames

        self.renderer = Renderer()
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)

        self.running = False
        self.paused = False
        self.dragging = False

        self.current = 0
        self.start = 0.0

    def run(self) -> None:
        if not self.frames:
            return

        self.running = True
        self.start = time.perf_counter()

        while self.running:
            now = time.perf_counter()

            self._handle_events(now)

            playback_time = now - self.start

            if not self.paused and not self.dragging:
                self._update(now)

            frame = self.frames[self.current]

            self.renderer.draw(frame)
            self._draw_overlay(frame)

            pygame.display.flip()
            self.clock.tick(self.FPS)

        pygame.quit()

    def _update(self, now: float) -> None:
        playback_time = now - self.start

        while self.current + 1 < len(self.frames) and self.frames[self.current + 1].time <= playback_time:
            self.current += 1

    def _handle_events(self, now: float) -> None:
        slider = self._slider()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                self._handle_keyboard(event, now)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if slider["hit"](event.pos):
                    self.dragging = True

            elif event.type == pygame.MOUSEBUTTONUP:
                self.dragging = False

            elif event.type == pygame.MOUSEMOTION and self.dragging:
                self.current = slider["update"](event.pos, len(self.frames))

                if not self.paused:
                    self.start = time.perf_counter() - self.frames[self.current].time

    def _handle_keyboard(self, event, now: float) -> None:

        if event.key == pygame.K_SPACE:
            self.paused = not self.paused

            if not self.paused:
                self.start = now - self.frames[self.current].time

        elif self.paused and event.key == pygame.K_RIGHT:
            self.current = min(
                self.current + int(1 / PHYSICS_DT),
                len(self.frames) - 1,
            )

        elif self.paused and event.key == pygame.K_LEFT:
            self.current = max(
                self.current - int(1 / PHYSICS_DT),
                0,
            )

    def _draw_overlay(self, frame: WorldFrame) -> None:

        screen = self.renderer.screen

        text = self.font.render(
            f"Time: {frame.time:.3f}s",
            True,
            (255, 255, 255),
        )
        screen.blit(text, (40, 40))

        status = self.font.render(
            "Paused" if self.paused else "Playing",
            True,
            (255, 255, 0),
        )
        screen.blit(status, (40, 65))

        self._draw_slider()

    def _draw_slider(self) -> None:

        slider = self._slider()

        pygame.draw.rect(
            self.renderer.screen,
            (90, 90, 90),
            (slider["x"], slider["y"], slider["width"], 6),
        )

        progress = self.current / (len(self.frames) - 1)

        knob_x = slider["x"] + progress * slider["width"]

        pygame.draw.circle(
            self.renderer.screen,
            (255, 255, 255),
            (int(knob_x), slider["y"] + 3),
            5,
        )

    def _slider(self):

        width, height = self.renderer.screen.get_size()

        margin = 50

        slider_y = height - 50
        slider_width = width - margin * 2

        def hit(pos):
            return abs(pos[1] - slider_y) < 20

        def update(pos, count):
            x = max(margin, min(pos[0], margin + slider_width))
            t = (x - margin) / slider_width
            return int(t * (count - 1))

        return {
            "x": margin,
            "y": slider_y,
            "width": slider_width,
            "hit": hit,
            "update": update,
        }
