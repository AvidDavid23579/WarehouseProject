import numpy as np

from common.types import NavPhase
from config import NUM_DOCKS
from navigation.graph import NavigationGraph
from simulator.world import World


def deliver_pallet(world: World, graph: NavigationGraph):
    robot = world.robot
    pallets = world.pallet

    for r in range(NUM_DOCKS):
        if robot.path[r]:
            continue

        available = np.flatnonzero(pallets.available)

        if available.size == 0:
            continue

        pallet_id = available[0]

        pallets.available[pallet_id] = False

        pallet_node = graph.pallet_nodes[pallet_id]
        goal_node = graph.goal_nodes[r]

        robot.goals[r, 0] = pallet_node
        robot.goals[r, 1] = goal_node
        robot.goal_index[r] = 0

        robot.path[r] = graph.dijkstra(
            robot.current_node_id[r],
            pallet_node,
        )

        robot.path_index[r] = 0
        robot.path_length[r] = len(robot.path[r])
        robot.nav_phase[r] = NavPhase.INITIAL_TURN
        robot.arrived[r] = False


def update_goal(world: World, graph: NavigationGraph) -> None:
    robot = world.robot

    for r in range(len(robot.pose)):
        if not robot.arrived[r]:
            continue

        if not robot.path[r]:
            continue

        goal_index = robot.goal_index[r] + 1

        # Entire sequence finished.
        if goal_index >= robot.goals.shape[1]:
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
