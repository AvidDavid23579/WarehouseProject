"""Simulation-wide constants for world geometry, entities, and control limits."""

import math

# --- World bounds (meters) ---------------------------------------------------
X_MIN = 0
X_MAX = 40
Y_MIN = 0
Y_MAX = 20

WORLD_BOUNDS = (X_MIN, X_MAX, Y_MIN, Y_MAX)

# --- Robot footprint ---------------------------------------------------------
ROBOT_WIDTH = 0.4
ROBOT_LENGTH = 0.6

# --- Shelf footprint ---------------------------------------------------------
SHELF_WIDTH = 0.6
SHELF_LENGTH = 6.0

# --- Goal / alignment tolerances ---------------------------------------------
DIST_TOLERANCE = 0.05
ANGLE_TOLERANCE = math.radians(1.0)

# --- Velocity limits ---------------------------------------------------------
MAX_VELOCITY = 8
MAX_OMEGA = 16
