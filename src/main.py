"""Entry point for the warehouse multi-robot simulator."""

import argparse
from pathlib import Path

import numpy as np

from config import ROBOT_LENGTH, WORLD_BOUNDS
from entities.robot import Pose, Robot
from entities.shelves import Shelf
from entities.wall import Wall
from rendering.replay_view import ReplayView
from simulator.exporter import export_video
from simulator.recorder import Recorder, Recording
from simulator.simulation import run_simulation
from simulator.snapshot import StaticWorld
from simulator.world import World

# --- Demo scenario layout ----------------------------------------------------
NUM_SHELVES = 9
NUM_ROBOTS = 38
PHYSICS_DT = 0.01
RECORDING_FPS = 30
SIM_DURATION = 100.0
DEFAULT_RECORDING_PATH = Path("recordings/latest.pkl")
DEFAULT_VIDEO_PATH = Path("recordings/latest.mp4")


def create_walls() -> list[Wall]:
    """Create a simple warehouse-style obstacle layout."""
    return []


def create_shelves(count: int) -> list[Shelf]:
    """Place shelves in a row along y = 10, spaced 4 m apart."""
    shelves = []
    for i in range(count):
        if i % 2 == 0:
            shelves.append(Shelf(shelf_id=i, pose=Pose(4 * (i + 1), 14, np.pi / 2)))
        else:
            shelves.append(Shelf(shelf_id=i, pose=Pose(4 * (i + 1), 6, np.pi / 2)))
    return shelves


def create_robots(count: int) -> list[Robot]:
    """Spawn robots along the bottom edge, each with waypoint goals."""
    robots = []
    for i in range(count):
        if i % 2 == 0:
            start = Pose(1 * (i + 1) + 1, ROBOT_LENGTH / 2 + 0.2, np.pi / 2)
            goals = [
                Pose(1 * (i + 1) + 1, ROBOT_LENGTH / 2 + 18, np.pi / 2),
                Pose(1 * (i + 1) + 1, ROBOT_LENGTH / 2 + 0.2, np.pi / 2),
            ]
        else:
            start = Pose(1 * (i + 1) - 1, ROBOT_LENGTH / 2 + 18, -np.pi / 2)
            goals = [
                Pose(1 * (i + 1) - 1, ROBOT_LENGTH / 2 + 0.2, -np.pi / 2),
                Pose(1 * (i + 1) - 1, ROBOT_LENGTH / 2 + 18, -np.pi / 2),
            ]
        robots.append(Robot(start, goals, robot_id=i))
    return robots


def build_world() -> World:
    """Assemble the demo world with shelves and robots."""
    world = World(bounds=WORLD_BOUNDS)
    for robot in create_robots(NUM_ROBOTS):
        world.add_robot(robot)
    for shelf in create_shelves(NUM_SHELVES):
        world.add_shelf(shelf)
    for wall in create_walls():
        world.add_wall(wall)
    return world


def simulate() -> Recording:
    """Phase 1: run physics and record dynamic snapshots (no rendering)."""
    world = build_world()
    static_world = StaticWorld.from_world(world)
    recorder = Recorder(
        recording_fps=RECORDING_FPS,
        physics_dt=PHYSICS_DT,
        static_world=static_world,
    )

    run_simulation(
        world,
        recorder,
        physics_dt=PHYSICS_DT,
        duration=SIM_DURATION,
    )

    recording = recorder.finish()
    print(f"Simulation complete: {recording.frame_count} frames ({recording.duration:.1f}s at {RECORDING_FPS} FPS recording)")
    return recording


def replay(recording: Recording) -> None:
    """Phase 2: interactive playback from a saved or freshly recorded run."""
    ReplayView(recording).run()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Warehouse simulator and replay")
    parser.add_argument(
        "--replay",
        type=Path,
        help="Load and replay a saved recording without running simulator",
    )
    parser.add_argument(
        "--simulate-only",
        action="store_true",
        help="Run simulator and save recording without opening replay",
    )
    parser.add_argument(
        "--export",
        type=Path,
        metavar="PATH",
        help="Export a recording to MP4 (use with --replay or after simulate)",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=DEFAULT_RECORDING_PATH,
        help=f"Path to save recording (default: {DEFAULT_RECORDING_PATH})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.replay:
        recording = Recording.load(args.replay)
        if args.export:
            export_video(recording, args.export)
            print(f"Exported video to {args.export}")
            return
        replay(recording)
        return

    recording = simulate()
    recording.save(args.save)
    print(f"Recording saved to {args.save}")

    if args.export:
        export_video(recording, args.export)
        print(f"Exported video to {args.export}")

    if not args.simulate_only:
        replay(recording)


if __name__ == "__main__":
    main()
