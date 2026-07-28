from dataclasses import dataclass


@dataclass(slots=True)
class Camera:
    scene_rect: tuple[int, int, int, int]
    world_width: float
    world_height: float

    @property
    def pixels_per_meter(self) -> float:
        return self.scene_rect[2] / self.world_width

    def world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        px, py, _, ph = self.scene_rect

        sx = px + x * self.pixels_per_meter
        sy = py + ph - y * self.pixels_per_meter

        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> tuple[float, float]:
        px, py, _, ph = self.scene_rect

        x = (sx - px) / self.pixels_per_meter
        y = (py + ph - sy) / self.pixels_per_meter

        return x, y

    def length_to_pixels(self, length: float) -> float:
        return length * self.pixels_per_meter
