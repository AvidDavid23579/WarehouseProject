import math

from common.types import Pose

# Warehouse parameters
SHELF_COL = 4
SHELF_ROW = 2
NUM_DOCKS = 1
NUM_PALLETS_PER_SHELF = 8

# --- Simulation parameters ---------------------------------------------------
PHYSICS_DT = 1 / 60
SIMULATION_DURATION = 100  # Seconds
WINDOW_WIDTH = 1540
WINDOW_HEIGHT = 800
WAREHOUSE_WIDTH = 1300  # Pixels
CELL_SIZE = 1.0  # Meters


# --- World bounds (meters) ---------------------------------------------------
X_MIN = 0
X_MAX = 30
Y_MIN = 0
Y_MAX = 15

WORLD_BOUNDS = (X_MIN, X_MAX, Y_MIN, Y_MAX)

# --- Robot footprint ---------------------------------------------------------
ROBOT_WIDTH = 0.4
ROBOT_LENGTH = 0.6

# --- Shelf footprint ---------------------------------------------------------
PALLET_WIDTH = 0.3
PALLET_LENGTH = 0.4
SHELF_WIDTH = 2 * PALLET_WIDTH
SHELF_LENGTH = NUM_PALLETS_PER_SHELF * PALLET_LENGTH

# --- Dock footprint ----------------------------------------------------------
DOCK_WIDTH = 0.3
DOCK_LENGTH = 0.5
DOCK_APPROACH_DISTANCE = 0.4
DOCK_SPACING = 1.0
DOCK_ONE_POSE = Pose(1.0, DOCK_WIDTH / 2, 0.0)
ROBOT_DOCK_DIST = 0.5

# --- Wall footprint ----------------------------------------------------------
WALL_WIDTH = 0.3

# --- Goal / alignment tolerances ---------------------------------------------
DIST_TOLERANCE = 0.03
ANGLE_TOLERANCE = math.radians(0.5)

# --- Velocity limits ---------------------------------------------------------
MAX_VELOCITY = 4
MAX_OMEGA = 8
