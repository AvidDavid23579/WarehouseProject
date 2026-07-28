import math
import time

import pygame

from config import ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, Y_MAX
from renderer.camera import Camera
from simulator.world import WorldState


class Renderer:
    WIDTH = 1540
    HEIGHT = 800
    FPS = 60

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)

        pygame.display.set_caption("Warehouse Playback")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)

    def playback(self, states: list[WorldState]):

        if not states:
            return

        running = True
        paused = False
        dragging = False
        current = 0

        start = time.perf_counter()

        while running:
            now = time.perf_counter()

            slider = self.get_slider()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    paused, start, current = self.handle_keyboard(event, paused, start, now, current, states)

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if slider["hit"](event.pos):
                        dragging = True

                elif event.type == pygame.MOUSEBUTTONUP:
                    dragging = False

                elif event.type == pygame.MOUSEMOTION and dragging:
                    current = slider["update"](event.pos, len(states))

                    if not paused:
                        start = time.perf_counter() - states[current].time

            if not paused and not dragging:
                playback_time = now - start

                while current + 1 < len(states) and states[current + 1].time <= playback_time:
                    current += 1

            self.screen.fill((30, 30, 30))
            state = states[current]
            self.draw_scene(state, X_MAX, Y_MAX)
            self.draw_ui(state, paused, current, len(states))
            pygame.display.flip()
            self.clock.tick(self.FPS)

        pygame.quit()

    @staticmethod
    def handle_keyboard(event, paused, start, now, current, states):
        if event.key == pygame.K_SPACE:
            paused = not paused

            # reset playback clock when resuming
            if not paused:
                start = now - states[current].time

        elif paused and event.key == pygame.K_RIGHT:
            current = min(current + 1, len(states) - 1)

        elif paused and event.key == pygame.K_LEFT:
            current = max(current - 1, 0)

        return paused, start, current

    def get_slider(self):

        margin = 50

        slider_y = self.HEIGHT - 50
        slider_width = self.WIDTH - margin * 2

        def hit(pos):
            return abs(pos[1] - slider_y) < 20

        def update(pos, count):

            x = max(margin, min(pos[0], margin + slider_width))

            t = (x - margin) / slider_width

            return int(t * (count - 1))

        return {"x": margin, "y": slider_y, "width": slider_width, "hit": hit, "update": update}

    def draw_ui(self, state, paused, current, total):

        text = self.font.render(f"Time: {state.time:.3f}s", True, (255, 255, 255))
        self.screen.blit(text, (40, 40))

        status = "Paused" if paused else "Playing"
        status_text = self.font.render(status, True, (255, 255, 0))

        self.screen.blit(status_text, (40, 65))

        dimensions = f"{X_MAX}m x {Y_MAX}m"
        dimensions_text = self.font.render(dimensions, True, (255, 255, 0))
        self.screen.blit(dimensions_text, (40, 90))

        self.draw_slider(current, total)

    def draw_slider(self, current, total):

        slider = self.get_slider()

        pygame.draw.rect(self.screen, (90, 90, 90), (slider["x"], slider["y"], slider["width"], 6))

        progress = current / (total - 1)

        knob_x = slider["x"] + progress * slider["width"]

        pygame.draw.circle(self.screen, (255, 255, 255), (int(knob_x), slider["y"] + 3), 5)

    def draw_scene(self, state, width, height):

        screen_width, screen_height = self.screen.get_size()

        # Fixed scene width in pixels
        scene_width_px = 1200

        # Preserve world aspect ratio
        scene_height_px = scene_width_px * (height / width)

        scene_x = (screen_width - scene_width_px) // 2
        scene_y = (screen_height - scene_height_px) // 2

        scene_rect = (
            int(scene_x),
            int(scene_y),
            int(scene_width_px),
            int(scene_height_px),
        )

        self.camera = Camera(scene_rect, width, height)

        # Background
        pygame.draw.rect(self.screen, (45, 45, 45), scene_rect)

        self.draw_grid(scene_rect, width, height, 2)

        for robot in state.robots:
            self.draw_robot(robot)

    def draw_grid(self, scene_rect, width, height, step):

        x0, y0, px_width, px_height = scene_rect

        pixels_per_meter = px_width / width

        # Vertical lines
        for x in range(0, int(width) + 1, step):
            px = int(round(x0 + x * pixels_per_meter))
            pygame.draw.line(
                self.screen,
                (70, 70, 70),
                (px, y0),
                (px, y0 + px_height),
                1,
            )

        # Horizontal lines
        for y in range(0, int(height) + 1, step):
            py = int(round(y0 + px_height - y * pixels_per_meter))
            pygame.draw.line(
                self.screen,
                (70, 70, 70),
                (x0, py),
                (x0 + px_width, py),
                1,
            )

    def draw_robot(self, robot):
        cx, cy = self.camera.world_to_screen(robot.state.pose.x, robot.state.pose.y)

        L = self.camera.length_to_pixels(ROBOT_LENGTH)
        W = self.camera.length_to_pixels(ROBOT_WIDTH)

        hl = L / 2
        hw = W / 2

        corners = [
            (+hl, +hw),
            (+hl, -hw),
            (-hl, -hw),
            (-hl, +hw),
        ]

        c = math.cos(robot.state.pose.theta)
        s = math.sin(robot.state.pose.theta)

        pts = []

        for dx, dy in corners:
            rx = dx * c - dy * s
            ry = dx * s + dy * c

            pts.append((cx + rx, cy - ry))  # subtract because screen y is inverted

        pygame.draw.polygon(self.screen, (0, 170, 255), pts)
