#!/usr/bin/env python3
"""AdachiExplorerのレンダリング確認用スクリプト"""

from __future__ import annotations

from search.micromouse_algorithms import AdachiExplorer


goals = [(0,7), (1,7), (2,7), (3,7), (4,7), (5,7), (6,7), (7,7),
         (8,7), (9,7), (10,7), (11,7), (12,7), (13,7), (14,7), (15,7)]


def main() -> int:
    explorer = AdachiExplorer(goals=goals)
    maze_map = explorer.render_text(show_goal=goals[0], show_distance=True)
    print(maze_map)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
