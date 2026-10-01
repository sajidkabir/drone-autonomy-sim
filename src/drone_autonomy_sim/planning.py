"""A* path planning on a grid.

Connectivity is 8-way: the drone may move to any of the eight neighboring
cells. Orthogonal moves cost 1.0 and diagonal moves cost sqrt(2). Diagonal
moves may not cut corners: a diagonal step from ``(x, y)`` to
``(x + dx, y + dy)`` is allowed only when both orthogonal neighbors
``(x + dx, y)`` and ``(x, y + dy)`` are free, so the path never clips the
corner of an obstacle.

The heuristic is the octile distance, which is admissible under this cost
model, so the returned path is a true shortest path on the grid.
"""

from __future__ import annotations

import heapq
import itertools
import math

from .world import Cell, GridWorld

SQRT2 = math.sqrt(2.0)

_DIRECTIONS = [
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
]


def move_cost(a: Cell, b: Cell) -> float:
    """Cost of one step between adjacent cells."""
    if a[0] != b[0] and a[1] != b[1]:
        return SQRT2
    return 1.0


def path_cost(path: list[Cell]) -> float:
    """Total cost of a path, summing the per-step move costs."""
    return sum(move_cost(a, b) for a, b in zip(path, path[1:]))


def _heuristic(a: Cell, b: Cell) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def _neighbors(world: GridWorld, cell: Cell):
    x, y = cell
    for dx, dy in _DIRECTIONS:
        nxt = (x + dx, y + dy)
        if not world.is_free(nxt):
            continue
        if dx != 0 and dy != 0:
            # No corner cutting through diagonal obstacles.
            if not world.is_free((x + dx, y)) or not world.is_free((x, y + dy)):
                continue
        yield nxt, move_cost(cell, nxt)


def find_path(world: GridWorld, start: Cell, goal: Cell) -> list[Cell] | None:
    """Shortest path from start to goal, inclusive of both ends.

    Returns None when no route exists, or when either endpoint is out of
    bounds or occupied.
    """
    if not world.is_free(start) or not world.is_free(goal):
        return None
    if start == goal:
        return [start]

    counter = itertools.count()
    open_heap: list[tuple[float, int, Cell]] = [
        (_heuristic(start, goal), next(counter), start)
    ]
    g_score: dict[Cell, float] = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in closed:
            continue
        closed.add(current)
        for nxt, step_cost in _neighbors(world, current):
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative
                came_from[nxt] = current
                heapq.heappush(
                    open_heap,
                    (tentative + _heuristic(nxt, goal), next(counter), nxt),
                )
    return None
