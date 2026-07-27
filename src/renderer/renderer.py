import time

import pygame

from simulator.world import WorldState


class Renderer:
    WIDTH = 1280
    HEIGHT = 720
    FPS = 60

    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        self.WIDTH, self.HEIGHT = self.screen.get_size()
        pygame.display.set_caption("Warehouse Playback")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 48)

    def playback(self, states: list[WorldState]):

        if not states:
            return

        running = True
        paused = False
        dragging = False

        current = 0

        slider_margin = 50
        slider_y = self.HEIGHT - 50
        slider_width = self.WIDTH - slider_margin * 2
        slider_height = 6
        knob_radius = 10

        start = time.perf_counter()
        fullscreen = True
        while running:
            now = time.perf_counter()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        fullscreen = not fullscreen

                        if fullscreen:
                            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                        else:
                            self.screen = pygame.display.set_mode((1280, 720))

                        self.WIDTH, self.HEIGHT = self.screen.get_size()

                        slider_y = self.HEIGHT - 50
                        slider_width = self.WIDTH - slider_margin * 2

                    elif event.key == pygame.K_SPACE:
                        paused = not paused

                        if not paused:
                            start = now - states[current].time

                    elif paused and event.key == pygame.K_RIGHT:
                        current = min(current + 1, len(states) - 1)

                    elif paused and event.key == pygame.K_LEFT:
                        current = max(current - 1, 0)

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos

                    if abs(my - slider_y) < 20:
                        dragging = True

                elif event.type == pygame.MOUSEBUTTONUP:
                    dragging = False

                    if not paused:
                        start = time.perf_counter() - states[current].time

                elif event.type == pygame.MOUSEMOTION and dragging:
                    mx = max(slider_margin, min(event.pos[0], slider_margin + slider_width))

                    t = (mx - slider_margin) / slider_width
                    current = int(t * (len(states) - 1))

                    if not paused:
                        start = time.perf_counter() - states[current].time

            if not paused and not dragging:
                playback_time = now - start

                while current + 1 < len(states) and states[current + 1].time <= playback_time:
                    current += 1

            self.screen.fill((30, 30, 30))

            state = states[current]

            text = self.font.render(
                f"Time: {state.time:.3f}s",
                True,
                (255, 255, 255),
            )

            self.screen.blit(text, (40, 40))

            status = "Paused" if paused else "Playing"

            status_text = self.font.render(
                status,
                True,
                (255, 255, 0),
            )

            self.screen.blit(status_text, (40, 90))

            # slider
            pygame.draw.rect(
                self.screen,
                (90, 90, 90),
                (slider_margin, slider_y, slider_width, slider_height),
            )

            progress = current / (len(states) - 1)

            knob_x = slider_margin + progress * slider_width

            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (int(knob_x), slider_y + slider_height // 2),
                knob_radius,
            )

            pygame.display.flip()

            self.clock.tick(self.FPS)

        pygame.quit()
