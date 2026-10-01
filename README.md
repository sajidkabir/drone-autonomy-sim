# drone-autonomy-sim

[![CI](https://github.com/sajidkabir/drone-autonomy-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/sajidkabir/drone-autonomy-sim/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23077783.svg)](https://doi.org/10.5281/zenodo.23077783)

A simulator for **onboard autonomous decision-making for drones**: the
"thinking drone" problem. The drone is dropped into a 2D world it has not
seen. It perceives its surroundings through a limited sensor radius,
evaluates its options, and decides on the fly: follow the plan, replan
around something new, or abandon the mission and come home before the
battery becomes a one-way trip. Every decision it makes is written to an
explainable decision log with the reason attached, so a human can audit
the flight afterwards and answer the only question that matters in
autonomy: *why did it do that?*

This package is the autonomy companion to
[solar-uav-sim](https://github.com/sajidkabir/solar-uav-sim): that project
asks whether a solar drone can stay up; this one asks whether a drone can
think. It is built to be read, extended, and argued with: small modules,
an explicit state machine, honest limitations, and tests that check the
behavior, not just the plumbing.

## Features

- **Grid world model**: a rectangular world of free and obstacle cells
  that can change mid-mission when a scripted event drops a new obstacle
  into it.
- **A\* path planning**: 8-connectivity with octile heuristic and true
  shortest paths on the grid. Diagonal moves cannot cut corners through
  obstacles.
- **Sense-decide-act agent**: a `DroneAgent` that perceives only within a
  configurable sensor radius, plans on its own internal map (unknown cells
  assumed free), and acts one step at a time.
- **Explicit decision state machine**: TAKEOFF, NAVIGATE, REPLAN,
  RETURN_HOME, LAND, DONE, FAILED. No hidden behavior: every transition
  is a logged decision.
- **Dynamic replanning**: when a new obstacle appears on the planned path
  and enters sensor range, the agent stops, replans around it, evaluates
  whether the detour is still worth it, and continues or turns home.
- **Battery-aware return-to-home**: the agent continuously estimates the
  cost of flying home. If the battery falls to that cost plus a reserve
  margin, it abandons the mission and lands back at its start instead of
  dying mid-grid.
- **Explainable decision log**: every sense, plan, replan, evaluation,
  return, and landing is recorded as a (step, state, action, reason)
  entry.
- **CLI demo**: a built-in scenario with a wall, one gap, and a surprise
  obstacle, with the full decision log and an ASCII map of the flight.

## Installation

Requires Python 3.10 or newer. No third-party dependencies.

```bash
git clone https://github.com/sajidkabir/drone-autonomy-sim.git
cd drone-autonomy-sim
pip install -e .
```

For development (adds the test runner):

```bash
pip install -e . pytest
pytest -q
```

## Quickstart

### Command line

Run the built-in demo mission:

```bash
drone-autonomy run
```

```text
Drone autonomy demo mission
  World:         14 x 9 grid, wall with a gap at x=9, y=4
  Start:         (0, 4)   Goal: (13, 4)
  Battery:       45.0 units (reserve margin 2.0)
  Sensor radius: 3 cells
  Event:         surprise obstacle appears at (5, 4) on step 2

Mission result
  Outcome:       SUCCESS, goal reached
  Steps:         15
  Final battery: 31.2
  End position:  (13, 4)

Decision log
  step   0 [TAKEOFF    ] takeoff: took off from (0, 4); initial plan has 13 moves, cost 13.0; battery 45.0
  step   3 [NAVIGATE   ] sense: sensor sweep (radius 3) revealed new obstacle(s) at [(5, 4)]
  step   3 [REPLAN     ] replan: planned path is blocked by perceived obstacle at (5, 4); new route has 11 moves, cost 11.8
  step   3 [REPLAN     ] evaluate_options: continue to goal: route cost 11.8, battery 43.0, margin 31.2; safe return home is still possible; decision: continue
  step   7 [REPLAN     ] replan: planned path is blocked by perceived obstacle at (9, 5); new route has 7 moves, cost 7.4
  step  13 [NAVIGATE   ] arrive: reached the goal at (13, 4); landing
  step  14 [LAND       ] land: touchdown at the goal; mission complete

Map (S start, G goal, # obstacle, * path flown)
.........#....
.........#....
.........#....
....****.#....
S***.#..*****G
.........#....
.........#....
.........#....
.........#....
```

(The full log also records every move and every sensor sweep; the excerpt
above keeps the decisions.) Read the log top to bottom and the story tells
itself: the drone plans down the corridor, senses the surprise obstacle
at step 3, replans around it, evaluates the detour against its battery
and decides to continue, then replans a second time at step 7 when the
edge of the wall enters sensor range, threads the gap, and lands.

Two variations worth trying:

```bash
drone-autonomy run --battery 20
```

The same scenario with a small battery: partway out, the safe-return
guard fires and the drone abandons the mission, flies home, and lands
with charge to spare.

```bash
drone-autonomy run --sensor-radius 1.5
```

A shorter sensor radius means the surprise obstacle is seen later and
the detour gets tighter.

### Python API

```python
from drone_autonomy_sim import GridWorld, ObstacleEvent, run_mission

# A 12x7 world split by a wall at x=6 with a single gap at y=3.
world = GridWorld(12, 7, obstacles={(6, y) for y in range(7) if y != 3})

result = run_mission(
    world,
    start=(0, 3),
    goal=(11, 3),
    events=[ObstacleEvent(step=1, cell=(3, 3))],  # surprise obstacle
    battery=60.0,
)

print(result.success)        # True: replanned around the surprise
print(result.steps)          # steps taken, takeoff and landing included
print(result.final_battery)  # charge left at touchdown
for entry in result.decision_log:
    print(entry.step, entry.state, entry.action, entry.reason)
```

## How it works

| Module | Responsibility |
| --- | --- |
| `world.py` | The grid world: dimensions, obstacles, free/occupied queries |
| `planning.py` | A* search, move costs, path costs |
| `agent.py` | The autonomy stack: perception, the state machine, the battery guard |
| `simulation.py` | The mission runner and the scripted obstacle events |
| `cli.py` | Command-line interface and the demo scenario |

The decision loop, once per step:

1. **Sense**: every true obstacle within `sensor_radius` (Euclidean) of
   the drone is added to its internal map. The drone never sees further,
   and never forgets.
2. **Decide**, in priority order:
   - *Battery guard*: if the remaining battery is at or below the
     estimated cost of flying home plus the reserve margin, abandon the
     mission (RETURN_HOME).
   - *Path check*: if a newly perceived obstacle blocks the planned path,
     enter REPLAN and compute a new route on the internal map. If the
     new route costs more energy than the battery holds, return home
     instead; if no route to the goal exists at all, return home while
     that is still possible.
3. **Act**: move one cell along the plan and pay the move cost
   (1.0 orthogonal, sqrt(2) diagonal; the battery can never go negative).

Key relations:

- Move cost: orthogonal 1.0, diagonal sqrt(2)
- A* heuristic (octile): h = (dx + dy) + (sqrt(2) - 2) * min(dx, dy),
  admissible under the move costs, so plans are true shortest paths
- Safe-return rule: return home when battery <= cost home + reserve
  margin, where cost home is the A* path cost on the internal map

## Validation and sanity checks

The test suite (16 tests) checks behavior, not just plumbing:

- A* returns the known optimum in an open grid: corner to corner of a
  5x5 grid is 5 cells at cost 4 * sqrt(2).
- A* returns None when the goal is walled off, and never cuts a diagonal
  corner through a blocked cell.
- The agent reaches the goal in a static wall-and-gap world.
- When an obstacle drops onto the path mid-mission, a REPLAN decision is
  logged and the agent still arrives.
- With a small battery, a RETURN_HOME decision is logged and the agent
  lands back at its start instead of dying mid-grid.
- When the goal is genuinely unreachable, the agent gives up and comes
  home rather than wandering forever.
- Every decision log entry carries a non-empty reason, and the battery
  never goes negative in any scenario.

## Honest limitations

- 2D grid world: no altitude, no 3D obstacles, no terrain.
- Discrete steps and perfect localization: the drone always knows its
  own cell exactly; there is no sensor noise or state estimation.
- Obstacles are static once they appear; nothing moves except the drone.
- The battery model is linear: cost per move, no discharge curves, no
  wind, no payload effects.
- Single agent: no other traffic, no deconfliction, no communication.

These are deliberate. Each one is a clean extension point, listed below.

## Roadmap and room for exploration

Ideas are welcome. Roughly in order of expected value:

- **Continuous 2D world**: real coordinates, circular obstacles, and a
  sampling-based planner (RRT or RRT*) alongside A*.
- **3D grid and altitude layers**, with climb and descent energy costs.
- **Sensor realism**: noise, false negatives, and a probabilistic
  occupancy grid instead of a perfect internal map.
- **Moving obstacles and other traffic**, with prediction and right-of-way
  rules: the first step toward counter-drone and air-defense scenarios.
- **Multi-agent missions**: two or more drones sharing a map and dividing
  goals, then swarms.
- **Energy realism**: plug the propulsion and battery models from
  solar-uav-sim into the move costs.
- **Learned policies**: log the decision traces and train or evaluate a
  learned decider against the state machine baseline.
- **A visualizer**: replay a mission from its decision log as an
  animation, step by step, with the reasons on screen.

If you build one of these, open an issue or a pull request. Design notes
in the PR description are appreciated: what assumption changed, and what
it did to the decision log on the reference missions.

## Project structure

```text
src/drone_autonomy_sim/   the package (world, planning, agent,
                          simulation, cli)
tests/                    pytest suite, behavior checks included
examples/                 runnable example missions
.github/workflows/        CI: install and run the test suite on every push
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). The short version: fork, branch,
test, pull request. Every change should keep `pytest -q` green and should
not change the behavior on the reference missions without explaining why
in the PR.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## Citation

If you use this project in research, please cite the archived release:

Sajid Kabir Saji (2026). drone-autonomy-sim (v1.0.1) [Software]. Zenodo. https://doi.org/10.5281/zenodo.23077784

The concept DOI https://doi.org/10.5281/zenodo.23077783 always resolves to the latest version.

## License

MIT. See [LICENSE](LICENSE).

## Author

Sajid Kabir Saji, aeronautical engineer. Research interests: onboard
autonomous decision-making for UAVs and solar-electric flight endurance.
More at [sajidkabir.com](https://sajidkabir.com).
