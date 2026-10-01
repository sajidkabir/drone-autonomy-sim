# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/), and versions follow
[Semantic Versioning](https://semver.org/).

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
