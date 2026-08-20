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

def compress_collinear_path(
    graph: NavigationGraph,
    path: list[int],
) -> list[int]:
    """Remove intermediate nodes that lie on the same straight segment."""

    if len(path) <= 2:
        return path

    result = [path[0]]

    for i in range(1, len(path) - 1):
        a = graph.node_pose[path[i - 1]]
        b = graph.node_pose[path[i]]
        c = graph.node_pose[path[i + 1]]

        v1 = b[:2] - a[:2]
        v2 = c[:2] - b[:2]

        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = np.dot(v1, v2)

        # Keep the node if:
        #   1. The direction changes, or
        #   2. The path reverses direction.
        if abs(cross) > 1e-6 or dot <= 0:
            result.append(path[i])

    result.append(path[-1])

    return result

def set_robot_path(
    world: World,
    graph: NavigationGraph,
    r: int,
    target_node: int,
):
    robot = world.robot

    path = graph.dijkstra(robot.current_node_id[r], target_node)

    # path = compress_collinear_path(graph, path)

    robot.path[r] = path
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

def resolve_path_conflicts(
    world: World,
    graph: NavigationGraph,
) -> None:

    robot = world.robot

    for r in range(len(robot.pose)):

        if not robot.path[r]:
            continue

        # Actual physical position.
        start = int(robot.current_node_id[r])

        # Actual task destination.
        goal = int(robot.path[r][-1])

        # FULL planned paths.
        my_path = robot.path[r]

        for other in range(r):

            if not robot.path[other]:
                continue

            # FULL planned path of the lower-priority robot.
            other_path = robot.path[other]

            my_nodes = set(my_path)
            other_nodes = set(other_path)

            overlap = my_nodes & other_nodes

            if not overlap:
                continue

            print(
                f"\nCONFLICT R{r} vs R{other}"
                f"\n  R{r} path: {my_path}"
                f"\n  R{other} path: {other_path}"
                f"\n  overlap: {sorted(overlap)}"
                f"\n  R{r} current: {start}"
                f"\n  R{r} goal: {goal}"
            )

            # Higher-ID robot avoids every node in the
            # lower-ID robot's FULL planned path.
            blocked_nodes = other_nodes.copy()

            # We are physically already here, so Dijkstra
            # must be allowed to start from our current node.
            blocked_nodes.discard(start)

            new_path = graph.dijkstra(
                start,
                goal,
                blocked_nodes=blocked_nodes,
            )

            print(
                f"  blocked: {sorted(blocked_nodes)}"
                f"\n  Dijkstra: {new_path}"
            )

            if new_path is None:
                print(
                    f"  R{r}: NO REROUTE, keeping existing path"
                )
                continue

            new_overlap = set(new_path) & blocked_nodes

            if new_overlap:
                print(
                    f"  R{r}: INVALID REROUTE, "
                    f"overlap={sorted(new_overlap)}, "
                    f"keeping existing path"
                )
                continue

            new_path = compress_collinear_path(
                graph,
                new_path,
            )

            robot.path[r] = new_path
            robot.path_index[r] = 0
            robot.path_length[r] = len(new_path)
            robot.nav_phase[r] = NavPhase.INITIAL_TURN

            print(
                f"  R{r}: REROUTED -> {new_path}"
            )

            my_path = robot.path[r]

            break