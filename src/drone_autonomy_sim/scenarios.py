"""Named mission scenarios: larger worlds for the autonomy agent.

The built-in demo is a 14 by 9 world. Real missions are bigger, messier,
and longer, so this module adds a small library of named scenarios on
larger grids. Each scenario is defined as an ASCII map plus a list of
scripted surprise-obstacle events, so the whole world is readable at a
glance and the mission is fully reproducible: no random numbers at run
time.

Map legend (rows are given top row first, ``y = 0`` is the bottom row):

- ``#``  obstacle cell
- ``S``  start (exactly one)
- ``G``  goal (exactly one)
- ``.``  free cell

Use :func:`scenario_names` to list the scenarios and
:func:`get_scenario` to load one by name. The CLI exposes the same set:
``drone-autonomy scenarios`` lists them, ``drone-autonomy run
--scenario gauntlet`` flies one.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .simulation import ObstacleEvent, build_demo_scenario
from .world import Cell, GridWorld

_FREE = "."
_WALL = "#"
_START = "S"
_GOAL = "G"


def world_from_ascii(text: str) -> tuple[GridWorld, Cell, Cell]:
    """Parse an ASCII map into a world, a start cell, and a goal cell.

    Rows are read top row first; ``y = 0`` is the bottom row. Every row
    must have the same length, and there must be exactly one ``S`` and
    one ``G``. Anything that is not ``.``, ``#``, ``S``, or ``G`` raises
    ``ValueError``.
    """
    rows = [line for line in text.splitlines() if line.strip()]
    if not rows:
        raise ValueError("ascii map is empty")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError("ascii map rows have different lengths")
    height = len(rows)
    obstacles: set[Cell] = set()
    start: Cell | None = None
    goal: Cell | None = None
    for row_index, row in enumerate(rows):
        y = height - 1 - row_index
        for x, char in enumerate(row):
            cell = (x, y)
            if char == _FREE:
                continue
            if char == _WALL:
                obstacles.add(cell)
            elif char == _START:
                if start is not None:
                    raise ValueError("ascii map has more than one start")
                start = cell
            elif char == _GOAL:
                if goal is not None:
                    raise ValueError("ascii map has more than one goal")
                goal = cell
            else:
                raise ValueError(f"unknown map character: {char!r}")
    if start is None:
        raise ValueError("ascii map has no start cell (S)")
    if goal is None:
        raise ValueError("ascii map has no goal cell (G)")
    return GridWorld(width, height, obstacles), start, goal


@dataclass(frozen=True)
class Scenario:
    """One named mission: a world, a task, and its scripted surprises."""

    name: str
    title: str
    description: str
    world: GridWorld
    start: Cell
    goal: Cell
    events: tuple[ObstacleEvent, ...] = ()
    battery: float = 100.0
    sensor_radius: float = 3.0
    reserve_margin: float = 2.0


def _from_ascii(
    name: str,
    title: str,
    description: str,
    ascii_map: str,
    events: tuple[ObstacleEvent, ...],
    battery: float,
    sensor_radius: float = 3.0,
    reserve_margin: float = 2.0,
) -> Scenario:
    world, start, goal = world_from_ascii(ascii_map)
    for event in events:
        if not world.is_free(event.cell):
            raise ValueError(
                f"scenario {name}: event cell {event.cell} is not free"
            )
    return Scenario(
        name=name,
        title=title,
        description=description,
        world=world,
        start=start,
        goal=goal,
        events=events,
        battery=battery,
        sensor_radius=sensor_radius,
        reserve_margin=reserve_margin,
    )


_GAUNTLET_MAP = """\
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
................#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.................
........#.......#.......#.........
.S......#.......#.......#.......G.
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#...............#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........
........#.......#.......#.........\
"""

_LONG_HAUL_MAP = """\
....................................................#.......
.........................#.....#.....#....#...#...#.........
.........#.###..........#....#...........##.................
.........#..#..#.........#..................................
...............#.......#..#................##.......#.......
.S.........#............#.............#.##................G.
................#.#.......#..........#............#.........
.............................#..............................
.............#...............................#....#.........
.....................#....#..............#.......#..........
.........##..........#..........#.......#.......#..#........
..........#........##..........##.........##................\
"""

_CLUTTER_MAP = """\
......#..................#...#....#..#..
....#.......#..............#......#.....
...##.#.#........#........#........#....
........................................
....#..............#.............#......
...#...#......##....................##.#
............#...............#........#..
...#..........................#.........
...............................#........
..........#......#.#...#.........#......
..........#................#..#.........
.S...........#.##..#..................G.
........#..................#............
.................................#......
............#..........#.....#.....#....
........................#.......#.......
...###.#............#.........#.#.......
...............##..........#...#..#.....
...#.#.....................#.....#......
......#.....................#..#..##....
............#...#....#..................
......#.......##....##.................#
....#.#......#......#.............#...#.
..............#......#..#........##.....\
"""


def _demo() -> Scenario:
    world, start, goal, events, battery = build_demo_scenario()
    return Scenario(
        name="demo",
        title="Wall and gap (demo)",
        description=(
            "The original 14 by 9 mission: a wall with a single gap, "
            "one surprise obstacle on the flight path, and a battery "
            "sized so the detour forces a real continue-or-return "
            "evaluation."
        ),
        world=world,
        start=start,
        goal=goal,
        events=tuple(events),
        battery=battery,
    )


def _gauntlet() -> Scenario:
    return _from_ascii(
        name="gauntlet",
        title="Three staggered walls",
        description=(
            "A 34 by 20 world split by three walls with offset gaps. "
            "The drone must thread every gap while a surprise obstacle "
            "drops onto its flight path mid-mission, so the flight is "
            "a chain of sense, replan, and evaluate cycles."
        ),
        ascii_map=_GAUNTLET_MAP,
        events=(ObstacleEvent(step=2, cell=(7, 11)),),
        battery=130.0,
    )


def _long_haul() -> Scenario:
    return _from_ascii(
        name="long_haul",
        title="Long corridor, short battery",
        description=(
            "A 60 by 12 corridor with scattered obstacles and a "
            "battery that cannot reach the goal. The honest outcome "
            "is a safe return home: this scenario exercises the "
            "battery guard and the abort decision at range, not the "
            "planner."
        ),
        ascii_map=_LONG_HAUL_MAP,
        events=(ObstacleEvent(step=2, cell=(15, 4)),),
        battery=48.0,
    )


def _clutter() -> Scenario:
    return _from_ascii(
        name="clutter",
        title="Scattered obstacle field",
        description=(
            "A 40 by 24 field of scattered obstacles with one surprise "
            "on the flight path. A stress test for repeated replanning: "
            "the agent discovers the field a few cells at a time and "
            "keeps finding new routes through it."
        ),
        ascii_map=_CLUTTER_MAP,
        events=(ObstacleEvent(step=2, cell=(13, 13)),),
        battery=200.0,
    )


_BUILDERS = {
    "demo": _demo,
    "gauntlet": _gauntlet,
    "long_haul": _long_haul,
    "clutter": _clutter,
}


def scenario_names() -> list[str]:
    """Names of the built-in scenarios, in presentation order."""
    return list(_BUILDERS)


def get_scenario(name: str) -> Scenario:
    """Load a scenario by name. Raises KeyError for unknown names."""
    try:
        return _BUILDERS[name]()
    except KeyError:
        raise KeyError(
            f"unknown scenario {name!r}; "
            f"available: {', '.join(scenario_names())}"
        ) from None
