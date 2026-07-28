import pygame
import pygame.gfxdraw

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import ROBOT_LENGTH, ROBOT_WIDTH, WAREHOUSE_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH, X_MAX, Y_MAX
from entities.robot import RobotFrame
from render.camera import Camera
from simulator.world import WorldFrame, WorldMap


class Renderer:
    def __init__(self, world_map: WorldMap) -> None:
        pygame.init()
        self.font = pygame.font.SysFont(None, 18)

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

    @staticmethod
    def _draw_grid(surface, scene_rect, step: int = 2) -> None:
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

    def _draw_polygon(self, surface, vertices, fill, outline=None, aa=True):
        assert self.camera is not None

        pts = [self.camera.world_to_screen(x, y) for x, y in vertices]

        pygame.gfxdraw.filled_polygon(surface, pts, fill)

        if aa:
            pygame.gfxdraw.aapolygon(
                surface,
                pts,
                outline or fill,
            )

        elif outline is not None:
            pygame.draw.polygon(
                surface,
                outline,
                pts,
                2,
            )

    def _draw_robot(self, robot: RobotFrame) -> None:
        vertices = rotated_rectangle_vertices(
            Pose(robot.x, robot.y, robot.theta),
            ROBOT_LENGTH,
            ROBOT_WIDTH,
        )

        self._draw_polygon(
            self.screen,
            vertices,
            fill=(0, 170, 255),
        )

    def _draw_static(self, surface):
        for dock in self.world_map.docks:
            self._draw_polygon(
                surface,
                dock.vertices,
                fill=(110, 110, 110),
                outline=(40, 40, 40),
                aa=False,
            )

        for shelf in self.world_map.shelves:
            self._draw_polygon(
                surface,
                shelf.vertices,
                fill=(170, 120, 60),
                outline=(90, 60, 30),
                aa=False,
            )

        for wall in self.world_map.walls:
            self._draw_polygon(
                surface,
                wall.vertices,
                fill=(100, 100, 100),
                outline=(70, 70, 70),
                aa=False,
            )
        nodes = []

        for dock in self.world_map.docks:
            nodes.append(dock.approach_node)
            nodes.append(dock.dock_node)

        self._draw_nodes(surface, nodes)

    def _draw_nodes(self, surface, nodes):
        # Draw edges
        for node in nodes:
            x1, y1 = self.camera.world_to_screen(node.pose.x, node.pose.y)

            for neighbor in node.neighbors:
                x2, y2 = self.camera.world_to_screen(
                    neighbor.pose.x,
                    neighbor.pose.y,
                )

                pygame.draw.line(
                    surface,
                    (180, 180, 180),
                    (x1 + 1, y1),
                    (x2 + 1, y2),
                    1,
                )

        # Draw nodes
        for node in nodes:
            x, y = self.camera.world_to_screen(node.pose.x, node.pose.y)

            pygame.draw.circle(
                surface,
                (255, 0, 0),
                (x + 2, y),
                3,
            )

            label = self.font.render(str(node.id), True, (255, 255, 255))
            surface.blit(label, (x, y))
