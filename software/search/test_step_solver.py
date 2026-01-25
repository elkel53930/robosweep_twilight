#!/usr/bin/env python3
from __future__ import annotations

import argparse
from typing import Tuple

from micromouse_algorithms import AdachiExplorer, Direction, Pose, read_maze_from_text_file


def _abs_wall_from_map(
    known_walls, observed, x: int, y: int, d: Direction
) -> bool:
    # Treat out-of-bounds as wall.
    size = len(known_walls)
    dx_dy = {
        Direction.NORTH: (0, 1),
        Direction.EAST: (1, 0),
        Direction.SOUTH: (0, -1),
        Direction.WEST: (-1, 0),
    }
    dx, dy = dx_dy[d]
    nx, ny = x + dx, y + dy
    if not (0 <= nx < size and 0 <= ny < size):
        return True

    # If not observed in file, treat as open (shouldn't happen for full maps).
    if not observed[y][x][int(d)]:
        return False
    return bool(known_walls[y][x][int(d)])


def _relative_walls(known_walls, observed, pose: Pose) -> Tuple[bool, bool, bool]:
    left = _abs_wall_from_map(known_walls, observed, pose.x, pose.y, pose.heading.left())
    front = _abs_wall_from_map(known_walls, observed, pose.x, pose.y, pose.heading)
    right = _abs_wall_from_map(known_walls, observed, pose.x, pose.y, pose.heading.right())
    return left, front, right


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Step-by-step micromouse solver demo. Reads a maze text file and runs AdachiExplorer, "
            "printing the maze and robot state each step."
        )
    )
    ap.add_argument("maze", help="Path to maze text file (mm_maze_solver style)")
    ap.add_argument(
        "--max-steps",
        type=int,
        default=2000,
        help="Safety limit to stop if not reaching goal (default: 2000)",
    )
    args = ap.parse_args()

    known_walls, observed, goal, start_pose = read_maze_from_text_file(args.maze)

    explorer = AdachiExplorer(maze_size=len(known_walls))
    # Load the full map so the renderer can show walls; the algorithm will still work.
    explorer.load_maze(known_walls, observed, pose=start_pose)

    # If the file contains a goal marker, use it as the goal.
    if goal is not None:
        explorer.set_goals([goal])

    for step in range(args.max_steps):
        print("\n" + "=" * 60)
        print(f"Step: {step}")
        print(f"Pose: (x={explorer.pose.x}, y={explorer.pose.y}, heading={explorer.pose.heading.name})")
        if goal is not None:
            print(f"Goal: (x={goal[0]}, y={goal[1]})")
        print(explorer.render_text(show_goal=goal))

        if goal is not None and (explorer.pose.x, explorer.pose.y) == goal:
            print("Reached goal.")
            return 0

        left, front, right = _relative_walls(known_walls, observed, explorer.pose)
        explorer.decide_heading(left, front, right)

        try:
            input("Press Enter for next step (Ctrl+C to quit)... ")
        except KeyboardInterrupt:
            print("\nInterrupted.")
            return 130

    print("Max steps reached without reaching goal.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
