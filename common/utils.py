import numpy as np


def wrap_angle(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))
