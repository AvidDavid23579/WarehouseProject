import math

import pygame
import pygame.gfxdraw

from config import ROBOT_LENGTH, ROBOT_WIDTH, WAREHOUSE_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH, X_MAX, Y_MAX
from entities.robot import RobotFrame
from render.camera import Camera
from simulator.world import WorldFrame, WorldMap


class Renderer:
    def __init__(self, world_map: WorldMap) -> None:
        pygame.init()

        self.screen = pygame.display.set_mode(
            (WINDOW_WIDTH, WINDOW_HEIGHT),
            pygame.RESIZABLE,
        )

        pygame.display.set_caption("Warehouse Playback")

        self.camera: Camera | None = None
        self.world_map = world_map

        self.background = None
        self.last_scene_rect = None

    def _draw(self, frame: WorldFrame) -> None:
        self._draw_scene(frame)

    def _draw_scene(self, frame: WorldFrame) -> None:
        screen_width, screen_height = self.screen.get_size()

        scene_width_px = WAREHOUSE_WIDTH
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

        if self.background is None or scene_rect != self.last_scene_rect:
            self.background = pygame.Surface(self.screen.get_size())
            self.background.fill((30, 30, 30))

            pygame.draw.rect(
                self.background,
                (45, 45, 45),
                scene_rect,
            )

            self._draw_grid(self.background, scene_rect)
            self._draw_static(self.background)

            self.last_scene_rect = scene_rect

        self.screen.blit(self.background, (0, 0))

        for robot in frame.robots:
            self._draw_robot(robot)

    def _draw_grid(self, surface, scene_rect, step: int = 2) -> None:
        x0, y0, px_width, px_height = scene_rect

        pixels_per_meter = px_width / X_MAX

        for x in range(0, int(X_MAX) + 1, step):
            px = int(round(x0 + x * pixels_per_meter))

            pygame.draw.line(
                surface,
                (70, 70, 70),
                (px, y0),
                (px, y0 + px_height),
                1,
            )

        for y in range(0, int(Y_MAX) + 1, step):
            py = int(round(y0 + px_height - y * pixels_per_meter))

            pygame.draw.line(
                surface,
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

    def _draw_static(self, surface):
        assert self.camera is not None

        for dock in self.world_map.docks:
            pts = [self.camera.world_to_screen(v.x, v.y) for v in dock.vertices]

            pygame.draw.polygon(surface, (80, 180, 255), pts)
            pygame.draw.polygon(surface, (30, 60, 120), pts, 2)

        for shelf in self.world_map.shelves:
            pts = [self.camera.world_to_screen(v.x, v.y) for v in shelf.vertices]

            pygame.draw.polygon(surface, (170, 120, 60), pts)
            pygame.draw.polygon(surface, (90, 60, 30), pts, 2)
        """
        for wall in self.world_map.walls:
            pts = [self.camera.world_to_screen(v.x, v.y) for v in wall.vertices]

            pygame.draw.polygon(surface, (100, 100, 100), pts)
        """
