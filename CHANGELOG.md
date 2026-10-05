# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/), and versions follow
[Semantic Versioning](https://semver.org/).

## [1.1.0] - 2026-10-05

### Added

- Scenario library (`drone_autonomy_sim.scenarios`): four named,
  fully reproducible missions on larger grids, each defined as a
  readable ASCII map plus scripted surprise-obstacle events.
  `get_scenario(name)` loads one, `scenario_names()` lists them, and
  `world_from_ascii` parses the map format (`#` obstacle, `S` start,
  `G` goal, `.` free; `y = 0` is the bottom row).
- `gauntlet`: a 34 by 20 world split by three staggered walls with
  offset gaps, with a surprise obstacle dropped onto the flight path
  mid-mission (67 steps to the goal, verified).
- `long_haul`: a 60 by 12 corridor with scattered obstacles and a
  battery that cannot reach the goal; the honest outcome is a safe
  return home, exercising the battery guard and the abort decision at
  range.
- `clutter`: a 40 by 24 scattered obstacle field with one surprise on
  the flight path, a stress test for repeated replanning (39 steps to
  the goal, verified).
- CLI: `drone-autonomy run --scenario NAME` flies any scenario
  (default `demo`), and `drone-autonomy scenarios` lists the set with
  dimensions and descriptions.
- Test suite grows from 16 to 40: ASCII map parsing and rejection of
  malformed maps, per-scenario validity, verified outcomes for all
  four scenarios, determinism of every scenario, and CLI coverage.

## [1.0.0] - 2026-10-01

First stable release.

### Added

- Grid world model (`GridWorld`): dimensions, obstacle set, bounds
  checks, and free/occupied queries.
- A* path planning on the grid with 8-connectivity, an octile heuristic,
  and corner-cutting prevention on diagonal moves (`find_path`,
  `path_cost`).
- The autonomy stack (`DroneAgent`): a sense-decide-act loop with an
  explicit decision state machine (TAKEOFF, NAVIGATE, REPLAN,
  RETURN_HOME, LAND, DONE, FAILED), perception limited to a configurable
  sensor radius, and an internal map that assumes unseen cells are free.
- Dynamic replanning: a newly perceived obstacle on the planned path
  triggers a replan, and the agent evaluates the detour against its
  battery before deciding to continue or turn home.
- Battery-aware return-to-home: the mission is abandoned when the
  remaining battery falls to the estimated cost of flying home plus a
  reserve margin.
- Explainable decision log: every sense, plan, replan, evaluation,
  return, and landing recorded as a (step, state, action, reason)
  entry.
- Mission runner (`run_mission`) with scripted obstacle events and a
  `MissionResult` carrying the success flag, the path taken, the step
  count, the decision log, and the final battery.
- Command-line interface (`drone-autonomy run`) with the built-in demo
  scenario and an ASCII map of the flight.
- Test suite of 16 tests covering the planner and the agent behavior,
  plus GitHub Actions CI on Python 3.12.
- Example mission: the demo scenario flown through the Python API.
