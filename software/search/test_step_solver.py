#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Iterable, Tuple

from micromouse_algorithms import AdachiExplorer, Direction, Pose, read_maze_from_text_file


logger = logging.getLogger("test_step_solver")


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


def _configure_logging(verbosity: int) -> None:
    # verbosity: 0=WARNING, 1=INFO, 2+=DEBUG
    if verbosity <= 0:
        level = logging.WARNING
    elif verbosity == 1:
        level = logging.INFO
    else:
        level = logging.DEBUG

    logging.basicConfig(
        level=level,
        format="[%(levelname)s] %(message)s",
    )


def _iter_maze_paths(path: str) -> Iterable[Path]:
    p = Path(path)
    if p.is_dir():
        logger.info("Input is a directory: %s", p)
        for child in sorted(p.iterdir()):
            if child.is_file():
                yield child
            else:
                logger.debug("Skip non-file: %s", child)
    else:
        yield p


def _format_file_context(text: str, offset: int, *, context_lines: int = 2) -> str:
    """Return a human-readable excerpt with a caret at the given character offset."""
    if offset < 0:
        offset = 0
    if offset > len(text):
        offset = len(text)

    # Compute 1-based line/col
    line = text.count("\n", 0, offset) + 1
    last_nl = text.rfind("\n", 0, offset)
    col = offset - (last_nl + 1) + 1

    lines = text.splitlines()
    line_idx = line - 1
    start = max(0, line_idx - context_lines)
    end = min(len(lines), line_idx + context_lines + 1)

    out = [f"(at line {line}, col {col})"]
    for i in range(start, end):
        prefix = ">" if i == line_idx else " "
        out.append(f"{prefix}{i+1:4d}: {lines[i]}")
        if i == line_idx:
            caret_pos = max(0, col - 1)
            out.append(f"     {' ' * 6}  {' ' * caret_pos}^")
    return "\n".join(out)


def _extract_expected_found(msg: str) -> tuple[str | None, str | None]:
    # For messages like: "Expected '|' or space, found '\n'"
    if "Expected" not in msg or "found" not in msg:
        return None, None
    try:
        exp = msg.split("Expected", 1)[1].split(", found", 1)[0].strip()
        found = msg.rsplit("found", 1)[1].strip()
        return exp, found
    except Exception:
        return None, None


def _run_one(maze_path: Path, *, max_steps: int, auto: bool, debug_parse: bool) -> int:
    logger.info("Loading maze: %s", maze_path)

    raw_text: str | None = None
    if debug_parse:
        try:
            raw_text = maze_path.read_text(encoding="utf-8")
        except Exception:
            logger.debug("Failed to read raw text for debug: %s", maze_path, exc_info=True)

    try:
        known_walls, observed, goal, start_pose = read_maze_from_text_file(str(maze_path))
    except ValueError as e:
        # Provide richer context for parser failures.
        logger.error("Maze parse error: %s", e)
        if debug_parse and raw_text is not None:
            exp, found = _extract_expected_found(str(e))
            logger.error("Parse detail: expected=%s found=%s", exp, found)

            # Best-effort: locate the first occurrence of the found token near where it matters.
            # For newline-related errors, point at the first blank line in the file.
            offset = 0
            if found is not None and found.strip("\"'") == "\\n":
                offset = raw_text.find("\n\n")
                if offset < 0:
                    offset = raw_text.find("\n")
            logger.error("File context:\n%s", _format_file_context(raw_text, max(0, offset)))
        return 1

    explorer = AdachiExplorer(maze_size=len(known_walls))
    explorer.load_maze(known_walls, observed, pose=start_pose)

    if goal is not None:
        explorer.set_goals([goal])

    logger.info(
        "Start pose: (x=%d, y=%d, heading=%s)",
        explorer.pose.x,
        explorer.pose.y,
        explorer.pose.heading.name,
    )
    if goal is not None:
        logger.info("Goal: (x=%d, y=%d)", goal[0], goal[1])
    else:
        logger.warning("No goal marker 'G' found in maze file.")

    for step in range(max_steps):
        print("\n" + "=" * 60)
        print(f"File: {maze_path}")
        print(f"Step: {step}")
        print(
            f"Pose: (x={explorer.pose.x}, y={explorer.pose.y}, heading={explorer.pose.heading.name})"
        )
        if goal is not None:
            print(f"Goal: (x={goal[0]}, y={goal[1]})")
        print(explorer.render_text(show_goal=goal))

        if goal is not None and (explorer.pose.x, explorer.pose.y) == goal:
            print("Reached goal.")
            logger.info("Reached goal in %d steps: %s", step, maze_path)
            return 0

        left, front, right = _relative_walls(known_walls, observed, explorer.pose)

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "Walls(rel): left=%s front=%s right=%s", left, front, right
            )

        prev_pose = Pose(explorer.pose.x, explorer.pose.y, explorer.pose.heading)
        new_heading = explorer.decide_heading(left, front, right)

        # Verify the move was legal w.r.t. the *true* map.
        dx = explorer.pose.x - prev_pose.x
        dy = explorer.pose.y - prev_pose.y
        if (dx, dy) == (0, 1):
            attempted_dir = Direction.NORTH
        elif (dx, dy) == (1, 0):
            attempted_dir = Direction.EAST
        elif (dx, dy) == (0, -1):
            attempted_dir = Direction.SOUTH
        elif (dx, dy) == (-1, 0):
            attempted_dir = Direction.WEST
        else:
            logger.error(
                "Invalid step delta at step %d in %s: prev=(%d,%d) new=(%d,%d)",
                step,
                maze_path,
                prev_pose.x,
                prev_pose.y,
                explorer.pose.x,
                explorer.pose.y,
            )
            return 2

        attempted_wall = _abs_wall_from_map(
            known_walls, observed, prev_pose.x, prev_pose.y, attempted_dir
        )
        if attempted_wall:
            logger.error(
                "Illegal move into wall detected at step %d in %s: pose=(%d,%d,%s) move_dir=%s",
                step,
                maze_path,
                prev_pose.x,
                prev_pose.y,
                prev_pose.heading.name,
                attempted_dir.name,
            )
            return 2

        if auto:
            # In auto mode, avoid spamming prints; use logs for movement trace.
            logger.info(
                "Move: (%d,%d,%s) -> (%d,%d,%s)",
                prev_pose.x,
                prev_pose.y,
                prev_pose.heading.name,
                explorer.pose.x,
                explorer.pose.y,
                new_heading.name,
            )

        if not auto:
            try:
                input("Press Enter for next step (Ctrl+C to quit)... ")
            except KeyboardInterrupt:
                print("\nInterrupted.")
                return 130

    logger.warning("Max steps reached without reaching goal: %s", maze_path)
    print("Max steps reached without reaching goal.")
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Step-by-step micromouse solver demo. Reads a maze text file (or directory of files) "
            "and runs AdachiExplorer, printing the maze and robot state each step."
        )
    )
    ap.add_argument(
        "maze",
        help=(
            "Path to a maze text file or a directory containing maze text files "
            "(mm_maze_solver style)"
        ),
    )
    ap.add_argument(
        "--max-steps",
        type=int,
        default=2000,
        help="Safety limit per maze (default: 2000)",
    )
    ap.add_argument(
        "--auto",
        action="store_true",
        help="Run without waiting for Enter (prints all steps until goal/max-steps)",
    )
    ap.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="Increase log verbosity (-v: INFO, -vv: DEBUG)",
    )
    ap.add_argument(
        "--debug-parse",
        action="store_true",
        help="On maze parse errors, print additional context/excerpts to logs",
    )
    args = ap.parse_args()

    _configure_logging(args.verbose)

    overall_rc = 0
    unsolved: list[Path] = []

    for maze_path in _iter_maze_paths(args.maze):
        try:
            rc = _run_one(
                maze_path,
                max_steps=args.max_steps,
                auto=args.auto,
                debug_parse=args.debug_parse,
            )
        except Exception:
            logger.exception("Error while running maze: %s", maze_path)
            rc = 1

        if rc != 0:
            unsolved.append(maze_path)
            if overall_rc == 0:
                overall_rc = rc

        # Between files, pause in manual mode.
        if not args.auto and Path(args.maze).is_dir():
            try:
                input("Press Enter to continue to next maze... ")
            except KeyboardInterrupt:
                print("\nInterrupted.")
                return 130

    if unsolved:
        print("\nUnsolved mazes:")
        for p in unsolved:
            print(f"  - {p}")
    else:
        print("\nAll mazes solved.")

    return overall_rc


if __name__ == "__main__":
    raise SystemExit(main())
