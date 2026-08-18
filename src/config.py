import math

from common.types import Pose

# Warehouse parameters
SHELF_COL = 1
SHELF_ROW = 1
NUM_DOCKS = 1
NUM_PALLETS_PER_SHELF = 2

# --- Simulation parameters ---------------------------------------------------
PHYSICS_DT = 1 / 60
SIMULATION_DURATION = 100  # Seconds
WINDOW_WIDTH = 1540
WINDOW_HEIGHT = 800
WAREHOUSE_WIDTH = 1300  # Pixels
CELL_SIZE = 1.0  # Meters


# --- World bounds (meters) ---------------------------------------------------
X_MIN = 0
X_MAX = 20
Y_MIN = 0
Y_MAX = 10

GOAL_ZONE_LENGTH = X_MAX / 3
GOAL_ZONE_WIDTH = 1.0

WORLD_BOUNDS = (X_MIN, X_MAX, Y_MIN, Y_MAX)

# --- Robot footprint ---------------------------------------------------------
ROBOT_WIDTH = 0.4
ROBOT_LENGTH = 0.6

# --- Shelf footprint ---------------------------------------------------------
PALLET_WIDTH = 0.3
PALLET_LENGTH = ROBOT_WIDTH
SHELF_WIDTH = 2 * PALLET_WIDTH
SHELF_LENGTH = NUM_PALLETS_PER_SHELF * PALLET_LENGTH

# --- Dock footprint ----------------------------------------------------------
DOCK_WIDTH = 0.3
DOCK_LENGTH = 0.5
DOCK_APPROACH_DISTANCE = 0.4
DOCK_SPACING = 1.0  # Should be 1.0
DOCK_ONE_POSE = Pose(1.5, DOCK_WIDTH / 2, 0.0)
ROBOT_DOCK_DIST = 0.5

# --- Goal / alignment tolerances ---------------------------------------------
DIST_TOLERANCE = 0.01
ANGLE_TOLERANCE = math.radians(0.1)

# --- Velocity limits ---------------------------------------------------------
MAX_VELOCITY = 4
MAX_OMEGA = 8
