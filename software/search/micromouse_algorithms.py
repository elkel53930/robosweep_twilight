from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


MAZE_SIZE: int = 16


class Direction(IntEnum):
    """Absolute heading in the maze coordinate system.

    Coordinate system:
      - (0, 0) is the lower-left cell when viewing the maze facing North.
      - x increases to the East, y increases to the North.
    """

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def left(self) -> "Direction":
        return Direction((int(self) + 3) % 4)

    def right(self) -> "Direction":
        return Direction((int(self) + 1) % 4)

    def back(self) -> "Direction":
        return Direction((int(self) + 2) % 4)


DIR_VECTORS = {
    Direction.NORTH: (0, 1),
    Direction.EAST: (1, 0),
    Direction.SOUTH: (0, -1),
    Direction.WEST: (-1, 0),
}


@dataclass
class Pose:
    x: int = 0
    y: int = 0
    heading: Direction = Direction.NORTH


def _in_bounds(x: int, y: int, size: int = MAZE_SIZE) -> bool:
    return 0 <= x < size and 0 <= y < size


def read_maze_from_text_file(
    filename: str, *, maze_size: int = MAZE_SIZE
) -> tuple[
    list[list[list[bool]]],
    list[list[list[bool]]],
    tuple[int, int] | None,
    Pose,
]:
    """Read maze walls from a text file (Rust mm_maze_solver compatible).

    Expected ASCII format (size=16 example):
      - '+' pillars
      - '-' horizontal walls (cell North wall; bottom row encodes South boundary)
      - '|' vertical walls (cell West wall; right boundary encodes East)
      - 'S' start marker (ignored for parsing; start is (0,0) by convention here)
      - 'G' goal marker (returned as (x,y) if present)

    Returns:
      known_walls, observed, goal, start_pose

    Coordinate notes:
      - The text file is written from top (north) to bottom (south).
      - We convert that to the internal coordinate system where (0,0) is bottom-left.
      - Default start (if no 'S' in file) is top-left facing East.
    """

    with open(filename, "r", encoding="utf-8") as f:
        s = f.read()

    known_walls: list[list[list[bool]]] = [
        [[False] * 4 for _ in range(maze_size)] for _ in range(maze_size)
    ]
    observed: list[list[list[bool]]] = [
        [[False] * 4 for _ in range(maze_size)] for _ in range(maze_size)
    ]

    def mark_wall(x: int, y: int, d: Direction, exists: bool = True) -> None:
        known_walls[y][x][int(d)] = exists
        observed[y][x][int(d)] = True
        dx, dy = DIR_VECTORS[d]
        nx, ny = x + dx, y + dy
        if _in_bounds(nx, ny, maze_size):
            od = d.back()
            known_walls[ny][nx][int(od)] = exists
            observed[ny][nx][int(od)] = True

    # State machine ports mm_maze_solver/bin/src/reader.rs
    state = "Pillar"  # Pillar, HorizontalWall, VerticalWall, Cell
    row = 0
    col = 0
    goal: tuple[int, int] | None = None
    start_pose = Pose(0, maze_size - 1, Direction.EAST)

    for c in s:
        if state == "Pillar":
            if c == "+":
                state = "HorizontalWall"
            elif c in ("\n", "\r"):
                # tolerate stray newlines
                continue
            else:
                raise ValueError(f"Expected '+', found {c!r}")

        elif state == "HorizontalWall":
            if c == "-":
                if row == maze_size:
                    # bottom boundary line encodes SOUTH walls of y=0
                    if not (0 <= col < maze_size):
                        raise ValueError("Horizontal wall out of bounds")
                    mark_wall(col, 0, Direction.SOUTH, True)
                else:
                    # Horizontal wall lines come *above* a cell row in the ASCII.
                    # The first horizontal line (row=0) is the top boundary => NORTH walls of y=maze_size-1.
                    # After each cell line, row increments, so row=k corresponds to boundary above y=maze_size-1-k.
                    y_cell = (maze_size - 1) if row == 0 else (maze_size - 1 - row)
                    if not _in_bounds(col, y_cell, maze_size):
                        raise ValueError("Horizontal wall out of bounds")
                    mark_wall(col, y_cell, Direction.NORTH, True)
                col += 1
                state = "Pillar"
            elif c == " ":
                col += 1
                state = "Pillar"
            elif c == "\n":
                col = 0
                state = "VerticalWall"
            elif c == "\r":
                continue
            else:
                raise ValueError(f"Expected '-' or space, found {c!r}")

            if col > maze_size + 1:
                raise ValueError("Too many columns in maze")

        elif state == "VerticalWall":
            if c == "|":
                y_cell = maze_size - 1 - row
                if col == maze_size:
                    # right boundary -> east wall of last column (x=maze_size-1)
                    if not (0 <= y_cell < maze_size):
                        raise ValueError("Vertical wall out of bounds")
                    mark_wall(maze_size - 1, y_cell, Direction.EAST, True)
                else:
                    # west wall of (x=col, y=y_cell)
                    if not _in_bounds(col, y_cell, maze_size):
                        raise ValueError("Vertical wall out of bounds")
                    mark_wall(col, y_cell, Direction.WEST, True)
                col += 1
                state = "Cell"
            elif c == " ":
                col += 1
                state = "Cell"
            elif c == "\r":
                continue
            else:
                raise ValueError(f"Expected '|' or space, found {c!r}")

        elif state == "Cell":
            if c == " ":
                state = "VerticalWall"
            elif c == "G":
                state = "VerticalWall"
                # In ASCII, current machine.row counts from top to bottom.
                goal = (col - 1, (maze_size - 1 - row))
            elif c == "S":
                state = "VerticalWall"
                start_pose = Pose(col - 1, (maze_size - 1 - row), Direction.EAST)
            elif c == "\n":
                row += 1
                col = 0
                state = "Pillar"
            elif c == "\r":
                continue
            else:
                raise ValueError(f"Expected cell char (space/S/G) or newline, found {c!r}")

        else:
            raise RuntimeError("invalid parser state")

    return known_walls, observed, goal, start_pose


class BaseExplorer:
    """Base class providing pose handling and wall bookkeeping.

    This module intentionally keeps the interface small:
      - set_pose(x, y, heading)
      - decide_heading(left_wall, front_wall, right_wall) -> Direction

    Wall information is stored as *known* walls. Unknown walls are treated as open
    for Adachi's flood-fill distance computation, as is common in exploration.
    """

    def __init__(self, maze_size: int = MAZE_SIZE):
        if maze_size <= 0:
            raise ValueError("maze_size must be positive")
        self.size = maze_size
        self.pose = Pose(0, 0, Direction.NORTH)

        # known_walls[y][x][dir] = True if wall exists
        self.known_walls: List[List[List[bool]]] = [
            [[False] * 4 for _ in range(self.size)] for _ in range(self.size)
        ]

        # whether a wall has been observed for that edge (exists or not)
        self.observed: List[List[List[bool]]] = [
            [[False] * 4 for _ in range(self.size)] for _ in range(self.size)
        ]

    def load_maze(
        self,
        known_walls: list[list[list[bool]]],
        observed: list[list[list[bool]]] | None = None,
        *,
        pose: Pose | None = None,
    ) -> None:
        """Load maze wall data into this explorer.

        Args:
            known_walls: [y][x][dir] wall presence.
            observed: [y][x][dir] whether that edge was observed.
                     If omitted, every edge will be treated as observed.
            pose: Optionally set current pose.

        This is useful to initialize from `read_maze_from_text_file()`.
        """

        if len(known_walls) != self.size or any(len(r) != self.size for r in known_walls):
            raise ValueError("known_walls size mismatch")

        self.known_walls = [[[False] * 4 for _ in range(self.size)] for _ in range(self.size)]
        self.observed = [[[False] * 4 for _ in range(self.size)] for _ in range(self.size)]

        for y in range(self.size):
            for x in range(self.size):
                if len(known_walls[y][x]) != 4:
                    raise ValueError("known_walls must have 4 directions per cell")
                for di in range(4):
                    self.known_walls[y][x][di] = bool(known_walls[y][x][di])

        if observed is None:
            for y in range(self.size):
                for x in range(self.size):
                    for di in range(4):
                        self.observed[y][x][di] = True
        else:
            if len(observed) != self.size or any(len(r) != self.size for r in observed):
                raise ValueError("observed size mismatch")
            for y in range(self.size):
                for x in range(self.size):
                    if len(observed[y][x]) != 4:
                        raise ValueError("observed must have 4 directions per cell")
                    for di in range(4):
                        self.observed[y][x][di] = bool(observed[y][x][di])

        if pose is not None:
            self.set_pose(pose.x, pose.y, pose.heading)

    def set_pose(self, x: int, y: int, heading: Direction | int) -> None:
        """Force the current position and heading."""
        if isinstance(heading, int):
            heading = Direction(heading)
        if not _in_bounds(x, y, self.size):
            raise ValueError(f"pose out of bounds: {(x, y)}")
        self.pose = Pose(x, y, heading)

    def mark_wall(self, x: int, y: int, d: Direction, exists: bool) -> None:
        """Mark wall on cell (x,y) side d; also sets opposite on neighbor."""
        self.known_walls[y][x][int(d)] = exists
        self.observed[y][x][int(d)] = True
        dx, dy = DIR_VECTORS[d]
        nx, ny = x + dx, y + dy
        if _in_bounds(nx, ny, self.size):
            od = d.back()
            self.known_walls[ny][nx][int(od)] = exists
            self.observed[ny][nx][int(od)] = True

    def update_walls_from_relative(self, left: bool, front: bool, right: bool) -> None:
        x, y, h = self.pose.x, self.pose.y, self.pose.heading
        self.mark_wall(x, y, h.left(), left)
        self.mark_wall(x, y, h, front)
        self.mark_wall(x, y, h.right(), right)

    def _can_move_abs(self, x: int, y: int, d: Direction) -> bool:
        # Outside the maze boundary is always a wall.
        dx, dy = DIR_VECTORS[d]
        nx, ny = x + dx, y + dy
        if not _in_bounds(nx, ny, self.size):
            return False

        # If we *observed* a wall and it exists => blocked.
        if self.observed[y][x][int(d)] and self.known_walls[y][x][int(d)]:
            return False

        # Unknown is treated as open.
        return True

    def step_forward(self) -> None:
        dx, dy = DIR_VECTORS[self.pose.heading]
        nx, ny = self.pose.x + dx, self.pose.y + dy
        if not _in_bounds(nx, ny, self.size):
            raise RuntimeError("attempted to step outside maze")
        self.pose.x, self.pose.y = nx, ny

    def render_text(self, *, show_goal: tuple[int, int] | None = None) -> str:
        """Render current maze knowledge and pose to an ASCII map.

        Style is compatible with the reference reader:
          - '+' pillars
          - '-' north/south walls
          - '|' west/east walls
          - 'S' at (0,0)
          - pose marker: '^', '>', 'v', '<'
          - optional goal marker: 'G'

        Unknown (unobserved) edges are drawn as spaces.
        """

        # We draw from top row (y=size-1) down to 0 so that (0,0) is left-bottom.
        heading_char = {
            Direction.NORTH: '^',
            Direction.EAST: '>',
            Direction.SOUTH: 'v',
            Direction.WEST: '<',
        }

        def h_wall_at(x: int, y: int, north: bool) -> str:
            # north wall of cell (x,y) or south wall of cell (x,y)
            d = Direction.NORTH if north else Direction.SOUTH
            if not self.observed[y][x][int(d)]:
                return ' '
            return '-' if self.known_walls[y][x][int(d)] else ' '

        def v_wall_at(x: int, y: int, west: bool) -> str:
            d = Direction.WEST if west else Direction.EAST
            if not self.observed[y][x][int(d)]:
                return ' '
            return '|' if self.known_walls[y][x][int(d)] else ' '

        lines: list[str] = []

        for y in range(self.size - 1, -1, -1):
            # Row top boundary: + - + - ... +
            top = ['+']
            for x in range(self.size):
                top.append(h_wall_at(x, y, north=True))
                top.append('+')
            lines.append(''.join(top))

            # Cell content line: | c | c | ... |
            mid: list[str] = []
            for x in range(self.size):
                # west wall at this cell (or boundary)
                mid.append(v_wall_at(x, y, west=True) if x > 0 else v_wall_at(x, y, west=True))

                ch = ' '
                if (x, y) == (0, 0):
                    ch = 'S'
                if show_goal is not None and (x, y) == show_goal:
                    ch = 'G'
                if (x, y) == (self.pose.x, self.pose.y):
                    ch = heading_char[self.pose.heading]
                mid.append(ch)
            # right boundary wall from east wall of last cell
            mid.append(v_wall_at(self.size - 1, y, west=False))
            lines.append(''.join(mid))

        # Bottom boundary: use south wall of y=0 cells
        bottom = ['+']
        for x in range(self.size):
            bottom.append(h_wall_at(x, 0, north=False))
            bottom.append('+')
        lines.append(''.join(bottom))

        return '\n'.join(lines)

    def decide_heading(self, left_wall: bool, front_wall: bool, right_wall: bool) -> Direction:
        raise NotImplementedError


class AdachiExplorer(BaseExplorer):
    """Adachi (flood-fill) method.

    Common micromouse version:
      - Maintain a distance map (potential field) to a goal region.
      - At each step, move to neighboring cell with minimum distance.
      - Break ties with a direction priority (usually Front > Left > Right > Back).

    Notes:
      - Unknown walls are treated as open, making the planner optimistic.
      - After discovering walls, distances are recomputed.
    """

    def __init__(
        self,
        maze_size: int = MAZE_SIZE,
        goals: Optional[Sequence[Tuple[int, int]]] = None,
        direction_priority: Optional[Sequence[str]] = None,
    ):
        super().__init__(maze_size=maze_size)

        # Default goal: center 2x2 for 16x16 (classic micromouse).
        if goals is None:
            c = self.size // 2
            goals = ((c - 1, c - 1), (c, c - 1), (c - 1, c), (c, c))
        self.goals: List[Tuple[int, int]] = list(goals)

        # Tie-breaking in relative directions.
        # Use strings to keep it simple for callers.
        if direction_priority is None:
            direction_priority = ("front", "left", "right", "back")
        self.direction_priority = list(direction_priority)

        self.dist: List[List[int]] = [[10**9] * self.size for _ in range(self.size)]
        self._recompute_distance_map()

    def set_goals(self, goals: Sequence[Tuple[int, int]]) -> None:
        self.goals = list(goals)
        self._recompute_distance_map()

    def _neighbors_open(self, x: int, y: int) -> Iterable[Tuple[int, int]]:
        for d in Direction:
            if self._can_move_abs(x, y, d):
                dx, dy = DIR_VECTORS[d]
                nx, ny = x + dx, y + dy
                if _in_bounds(nx, ny, self.size):
                    yield nx, ny

    def _recompute_distance_map(self) -> None:
        # Multi-source BFS (uniform costs).
        INF = 10**9
        self.dist = [[INF] * self.size for _ in range(self.size)]

        from collections import deque

        q = deque()
        for (gx, gy) in self.goals:
            if not _in_bounds(gx, gy, self.size):
                raise ValueError(f"goal out of bounds: {(gx, gy)}")
            self.dist[gy][gx] = 0
            q.append((gx, gy))

        while q:
            x, y = q.popleft()
            nd = self.dist[y][x] + 1
            for nx, ny in self._neighbors_open(x, y):
                if self.dist[ny][nx] > nd:
                    self.dist[ny][nx] = nd
                    q.append((nx, ny))

    def _rel_to_abs(self, rel: str) -> Direction:
        h = self.pose.heading
        if rel == "front":
            return h
        if rel == "left":
            return h.left()
        if rel == "right":
            return h.right()
        if rel == "back":
            return h.back()
        raise ValueError(f"unknown rel direction: {rel}")

    def decide_heading(self, left_wall: bool, front_wall: bool, right_wall: bool) -> Direction:
        # 1) Update known walls for this cell.
        self.update_walls_from_relative(left_wall, front_wall, right_wall)

        # 2) Recompute distances with updated walls.
        self._recompute_distance_map()

        x, y, h = self.pose.x, self.pose.y, self.pose.heading

        # 3) Select move that reduces distance.
        best_abs: Optional[Direction] = None
        best_dist = 10**9

        for rel in self.direction_priority:
            d = self._rel_to_abs(rel)
            if not self._can_move_abs(x, y, d):
                continue
            dx, dy = DIR_VECTORS[d]
            nx, ny = x + dx, y + dy
            if not _in_bounds(nx, ny, self.size):
                continue
            dd = self.dist[ny][nx]
            if dd < best_dist:
                best_dist = dd
                best_abs = d

        # Fallback: if all blocked in known map (should be rare), turn back.
        if best_abs is None:
            best_abs = h.back()

        self.pose.heading = best_abs
        self.step_forward()
        return self.pose.heading
