"""Command-line interface for drone-autonomy-sim.

Usage:
    drone-autonomy run [--battery N] [--sensor-radius R]
                       [--reserve-margin M]

The demo mission: a wall with a single gap, a surprise obstacle dropped
onto the flight path mid-mission, and a battery sized so the agent has to
make real decisions. Try ``drone-autonomy run --battery 20`` to watch the
same scenario end with the drone abandoning the mission and coming home.
"""

from __future__ import annotations

import argparse

from .simulation import build_demo_scenario, run_mission
from .world import GridWorld


def _render_map(world: GridWorld, start, goal, path_taken) -> str:
    path_set = set(path_taken)
    rows = []
    for y in reversed(range(world.height)):
        row = ""
        for x in range(world.width):
            cell = (x, y)
            if cell == start:
                row += "S"
            elif cell == goal:
                row += "G"
            elif world.is_occupied(cell):
                row += "#"
            elif cell in path_set:
                row += "*"
            else:
                row += "."
        rows.append(row)
    return "\n".join(rows)


def _run_demo(args: argparse.Namespace) -> int:
    world, start, goal, events, battery = build_demo_scenario()
    if args.battery is not None:
        battery = args.battery

    print("Drone autonomy demo mission")
    print(f"  World:         {world.width} x {world.height} grid, "
          f"wall with a gap at x=9, y=4")
    print(f"  Start:         {start}   Goal: {goal}")
    print(f"  Battery:       {battery:.1f} units "
          f"(reserve margin {args.reserve_margin:.1f})")
    print(f"  Sensor radius: {args.sensor_radius:g} cells")
    for event in events:
        print(f"  Event:         surprise obstacle appears at "
              f"{event.cell} on step {event.step}")
    print()

    result = run_mission(
        world,
        start,
        goal,
        events,
        battery,
        sensor_radius=args.sensor_radius,
        reserve_margin=args.reserve_margin,
    )

    outcome = "SUCCESS, goal reached" if result.success else "GOAL NOT REACHED"
    print("Mission result")
    print(f"  Outcome:       {outcome}")
    print(f"  Steps:         {result.steps}")
    print(f"  Final battery: {result.final_battery:.1f}")
    print(f"  End position:  {result.end_position}")
    print()
    print("Decision log")
    for entry in result.decision_log:
        print(f"  step {entry.step:>3} [{entry.state:<11}] "
              f"{entry.action}: {entry.reason}")
    print()
    print("Map (S start, G goal, # obstacle, * path flown)")
    print(_render_map(world, start, goal, result.path_taken))
    return 0 if result.success else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="drone-autonomy",
        description="Onboard autonomous decision-making simulator for drones.",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run the built-in demo mission")
    run.add_argument("--battery", type=float, default=None,
                     help="starting battery in energy units "
                          "(default: demo value)")
    run.add_argument("--sensor-radius", type=float, default=3.0,
                     help="sensor radius in cells (default: 3)")
    run.add_argument("--reserve-margin", type=float, default=2.0,
                     help="battery reserve kept for the flight home "
                          "(default: 2)")
    args = parser.parse_args(argv)
    if args.command == "run":
        return _run_demo(args)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
