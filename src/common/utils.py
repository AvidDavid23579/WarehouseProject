def world_to_screen(self, x, y, scene_rect, meters_width, meters_height):
    px, py, pw, ph = scene_rect

    sx = px + (x / meters_width) * pw

    # Flip y because pygame y increases downward
    sy = py + ph - (y / meters_height) * ph

    return int(sx), int(sy)
