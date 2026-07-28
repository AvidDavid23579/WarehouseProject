import math

import pygame
import pygame.gfxdraw

from config import ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, Y_MAX
from entities.robot import RobotFrame
from render.camera import Camera
from simulator.world import WorldFrame


class Renderer:
    WIDTH = 1540
    HEIGHT = 800

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode(
            (self.WIDTH, self.HEIGHT),
            pygame.RESIZABLE,
        )

        pygame.display.set_caption("Warehouse Playback")

        self.camera: Camera | None = None

    def draw(self, frame: WorldFrame) -> None:
        """Draw a single frame."""

        self.screen.fill((30, 30, 30))

        self._draw_scene(frame)

    def _draw_scene(self, frame: WorldFrame) -> None:
        screen_width, screen_height = self.screen.get_size()

        scene_width_px = 1300
        scene_height_px = scene_width_px * (Y_MAX / X_MAX)

        scene_x = (screen_width - scene_width_px) // 2
        scene_y = (screen_height - scene_height_px) // 2

        scene_rect = (
            int(scene_x),
            int(scene_y),
            int(scene_width_px),
            int(scene_height_px),
        )

        self.camera = Camera(scene_rect, X_MAX, Y_MAX)

        pygame.draw.rect(self.screen, (45, 45, 45), scene_rect)

        self._draw_grid(scene_rect)

        for robot in frame.robots:
            self._draw_robot(robot)

    def _draw_grid(self, scene_rect, step: int = 2) -> None:
        x0, y0, px_width, px_height = scene_rect

        pixels_per_meter = px_width / X_MAX

        for x in range(0, int(X_MAX) + 1, step):
            px = int(round(x0 + x * pixels_per_meter))

            pygame.draw.line(
                self.screen,
                (70, 70, 70),
                (px, y0),
                (px, y0 + px_height),
                1,
            )

        for y in range(0, int(Y_MAX) + 1, step):
            py = int(round(y0 + px_height - y * pixels_per_meter))

            pygame.draw.line(
                self.screen,
                (70, 70, 70),
                (x0, py),
                (x0 + px_width, py),
                1,
            )

    def _draw_robot(self, robot: RobotFrame) -> None:
        assert self.camera is not None

        cx, cy = self.camera.world_to_screen(robot.x, robot.y)

        length = self.camera.length_to_pixels(ROBOT_LENGTH)
        width = self.camera.length_to_pixels(ROBOT_WIDTH)

        hl = length / 2
        hw = width / 2

        corners = [
            (+hl, +hw),
            (+hl, -hw),
            (-hl, -hw),
            (-hl, +hw),
        ]

        c = math.cos(robot.theta)
        s = math.sin(robot.theta)

        points = []

        for dx, dy in corners:
            rx = dx * c - dy * s
            ry = dx * s + dy * c

            points.append((cx + rx, cy - ry))

        color = (0, 170, 255)

        pygame.gfxdraw.filled_polygon(
            self.screen,
            points,
            color,
        )

        pygame.gfxdraw.aapolygon(
            self.screen,
            points,
            color,
        )
