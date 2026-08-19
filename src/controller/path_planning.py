import numpy as np

from common.types import NavPhase
from config import NUM_DOCKS, NUM_PALLETS, PALLET_LENGTH, PALLET_WIDTH
from navigation.graph import NavigationGraph
from simulator.world import World
from entities.pallet import PalletStatus
from geometry.geo_compute import obb_vertices


def update_goal(world: World, graph: NavigationGraph) -> None:
    robot = world.robot

    for r in range(len(robot.pose)):
        if not robot.arrived[r]:
            continue

        if not robot.path[r]:
            continue

        goal_index = robot.goal_index[r] + 1

        # Entire sequence finished.
        if goal_index >= robot.goals.shape[1] or robot.goals[r, goal_index] == -1:
            robot.path[r] = []
            robot.path_index[r] = 0
            robot.path_length[r] = 0
            continue

        robot.goal_index[r] = goal_index

        goal_node = robot.goals[r, goal_index]

        robot.path[r] = graph.dijkstra(
            robot.current_node_id[r],
            goal_node,
        )

        robot.path_index[r] = 0
        robot.path_length[r] = len(robot.path[r])
        robot.target_node_id[r] = goal_node
        robot.nav_phase[r] = NavPhase.INITIAL_TURN
        robot.arrived[r] = False

def set_robot_path(
    world: World,
    graph: NavigationGraph,
    r: int,
    target_node: int,
):
    robot = world.robot

    robot.path[r] = graph.dijkstra(
        robot.current_node_id[r],
        target_node,
    )

    robot.path_index[r] = 0
    robot.path_length[r] = len(robot.path[r])
    robot.nav_phase[r] = NavPhase.INITIAL_TURN
    robot.arrived[r] = False

def assign_pallets(world: World, graph: NavigationGraph):

    robot = world.robot
    pallets = world.pallet

    for r in range(NUM_DOCKS):

        # Robot already has a task.
        if robot.pallet_id[r] != -1:
            continue

        # Robot already has a path.
        if robot.path[r]:
            continue

        available = np.flatnonzero(
            pallets.status == PalletStatus.UNDELIVERED
        )

        if available.size == 0:
            continue

        pallet_id = available[0]

        pallets.status[pallet_id] = PalletStatus.RESERVED
        pallets.robot_id[pallet_id] = r

        robot.goals[r, 0] = graph.pallet_nodes[pallet_id]
        robot.goals[r, 1] = graph.goal_nodes[r]
        robot.goal_index[r] = 0

        set_robot_path(
            world,
            graph,
            r,
            graph.pallet_nodes[pallet_id],
        )

def handle_pickups(world: World, graph: NavigationGraph):

    robot = world.robot
    pallets = world.pallet

    for r in range(NUM_DOCKS):

        if not robot.arrived[r]:
            continue

        if robot.pallet_id[r] != -1:
            continue

        reserved = np.flatnonzero(
            (pallets.status == PalletStatus.RESERVED) &
            (pallets.robot_id == r)
        )

        if reserved.size == 0:
            continue

        pallet_id = reserved[0]

        # Pick up pallet
        pallets.status[pallet_id] = PalletStatus.DELIVERING
        robot.pallet_id[r] = pallet_id

        # Move to delivery goal
        robot.goal_index[r] = 1

        set_robot_path(
            world,
            graph,
            r,
            robot.goals[r, 1],
        )

def handle_deliveries(world: World, graph: NavigationGraph):

    robot = world.robot
    pallets = world.pallet

    for r in range(NUM_DOCKS):

        pallet_id = robot.pallet_id[r]

        if pallet_id == -1:
            continue

        if not robot.arrived[r]:
            continue

        if robot.goal_index[r] != 1:
            continue

        # Delivered
        pallets.status[pallet_id] = PalletStatus.DELIVERED
        pallets.robot_id[pallet_id] = -1

        # Free robot
        robot.pallet_id[r] = -1
        robot.goals[r] = -1
        robot.goal_index[r] = 0
        robot.target_node_id[r] = -1

        robot.path[r] = []
        robot.path_index[r] = 0
        robot.path_length[r] = 0
        robot.nav_phase[r] = NavPhase.DRIVE
        robot.arrived[r] = True

def update_carried_pallets(world: World):

    robot = world.robot
    pallets = world.pallet

    for p in range(NUM_PALLETS):

        if pallets.status[p] != PalletStatus.DELIVERING:
            continue

        r = pallets.robot_id[p]

        if r == -1:
            continue

        pallets.pose[p] = robot.pose[r]
        pallets.vertices[p] = obb_vertices(
            pallets.pose[p:p+1],
            PALLET_LENGTH,
            PALLET_WIDTH,
        )

def return_to_dock(world: World, graph: NavigationGraph):

    robot = world.robot
    pallets = world.pallet

    available = np.any(
        pallets.status == PalletStatus.UNDELIVERED
    )

    if available:
        return

    for r in range(NUM_DOCKS):

        # Robot isn't idle.
        if robot.pallet_id[r] != -1:
            continue

        # Already has somewhere to go.
        if robot.path[r]:
            continue

        dock_node = graph.dock_nodes[r]

        # Already at dock.
        if robot.current_node_id[r] == dock_node:
            robot.arrived[r] = True
            continue

        set_robot_path(
            world,
            graph,
            r,
            dock_node,
        )