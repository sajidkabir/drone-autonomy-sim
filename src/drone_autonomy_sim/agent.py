"""The autonomy stack: a drone agent with a sense-decide-act loop.

The agent is a "thinking drone" in miniature. It does not follow a fixed
script. Every step it:

1. **Senses**: obstacles within ``sensor_radius`` of its position are added
   to its internal map. Cells it has never seen are assumed free, which is
   what makes mid-mission surprises possible.
2. **Decides**: an explicit state machine evaluates the situation: follow
   the plan, replan around a newly perceived obstacle, or abandon the
   mission and return home because the battery no longer covers a safe
   return.
3. **Acts**: moves one cell along the current plan and pays the battery
   cost of the move.

Every decision is appended to an explainable decision log: a list of
(step, state, action, reason) entries a human can audit after the flight.

States: TAKEOFF, NAVIGATE, REPLAN, RETURN_HOME, LAND, DONE, FAILED.
REPLAN is a decision state entered for one step while a new route is
computed; the log records it so the moment of replanning stays visible.

The battery rule is the same one real return-to-launch failsafes use:
while navigating, if the remaining battery falls to or below the estimated
cost of flying home plus a reserve margin, the mission is abandoned and
the agent flies home instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import NamedTuple

from .planning import find_path, move_cost, path_cost
from .world import Cell, GridWorld


class AgentState(Enum):
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    REPLAN = "REPLAN"
    RETURN_HOME = "RETURN_HOME"
    LAND = "LAND"
    DONE = "DONE"
    FAILED = "FAILED"


class DecisionEntry(NamedTuple):
    """One auditable decision: when, in which state, what, and why."""

    step: int
    state: str
    action: str
    reason: str


@dataclass
class DroneAgent:
    """An autonomous drone flying a point-to-point mission on a grid."""

    start: Cell
    goal: Cell
    battery_capacity: float
    sensor_radius: float = 3.0
    reserve_margin: float = 2.0

    def __post_init__(self) -> None:
        self.position: Cell = self.start
        self.home: Cell = self.start
        self.battery: float = float(self.battery_capacity)
        self.state: AgentState = AgentState.TAKEOFF
        self.known_obstacles: set[Cell] = set()
        self.path: list[Cell] = []
        self.decision_log: list[DecisionEntry] = []
        self.path_taken: list[Cell] = [self.start]
        self.step_count: int = 0
        self.reached_goal: bool = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def finished(self) -> bool:
        return self.state in (AgentState.DONE, AgentState.FAILED)

    def step(self, world: GridWorld) -> None:
        """Run one sense-decide-act cycle against the true world."""
        if self.finished:
            return
        self._perceive(world)
        if self.state is AgentState.TAKEOFF:
            self._takeoff(world)
        elif self.state is AgentState.NAVIGATE:
            self._navigate(world, self.goal, mission=True)
        elif self.state is AgentState.RETURN_HOME:
            self._navigate(world, self.home, mission=False)
        elif self.state is AgentState.LAND:
            self._land()
        self.step_count += 1

    def abort(self, reason: str) -> None:
        """Fail the mission with an auditable reason."""
        self._log("abort", reason)
        self.state = AgentState.FAILED

    # ------------------------------------------------------------------
    # Sense
    # ------------------------------------------------------------------

    def _perceive(self, world: GridWorld) -> None:
        seen = {
            cell
            for cell in world.obstacles
            if math.dist(self.position, cell) <= self.sensor_radius
        }
        new = seen - self.known_obstacles
        if new:
            self.known_obstacles |= seen
            self._log(
                "sense",
                f"sensor sweep (radius {self.sensor_radius:g}) revealed "
                f"new obstacle(s) at {sorted(new)}",
            )

    def _known_world(self, world: GridWorld) -> GridWorld:
        return GridWorld(world.width, world.height, set(self.known_obstacles))

    def _plan(self, world: GridWorld, target: Cell) -> list[Cell] | None:
        return find_path(self._known_world(world), self.position, target)

    # ------------------------------------------------------------------
    # Decide
    # ------------------------------------------------------------------

    def _takeoff(self, world: GridWorld) -> None:
        path = self._plan(world, self.goal)
        if path is None:
            self.abort(
                "no route to the goal exists on the initial map; "
                "staying on the ground"
            )
            return
        self.path = path
        self._log(
            "takeoff",
            f"took off from {self.home}; initial plan has "
            f"{len(path) - 1} moves, cost {path_cost(path):.1f}; "
            f"battery {self.battery:.1f}",
        )
        self.state = AgentState.NAVIGATE

    def _navigate(self, world: GridWorld, target: Cell, mission: bool) -> None:
        if mission and self._battery_guard_tripped(world):
            reason = (
                f"battery {self.battery:.1f} is at or below the safe-return "
                f"threshold {self._return_cost(world) + self.reserve_margin:.1f} "
                f"(return cost {self._return_cost(world):.1f} + reserve "
                f"{self.reserve_margin:.1f}); abandoning the mission"
            )
            if not self._start_return_home(world, reason):
                return
            target, mission = self.home, False
        if not self._path_is_valid(target):
            if not self._replan(world, target, mission):
                return
            if self.state is AgentState.RETURN_HOME:
                target, mission = self.home, False
        self._follow_one_step(target, mission)

    def _battery_guard_tripped(self, world: GridWorld) -> bool:
        if self.position == self.home:
            return False
        threshold = self._return_cost(world) + self.reserve_margin
        return self.battery <= threshold

    def _return_cost(self, world: GridWorld) -> float:
        path = self._plan(world, self.home)
        if path is not None:
            return path_cost(path)
        # No known route home: fall back to the octile distance estimate.
        dx = abs(self.position[0] - self.home[0])
        dy = abs(self.position[1] - self.home[1])
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    def _path_is_valid(self, target: Cell) -> bool:
        return (
            len(self.path) >= 1
            and self.path[0] == self.position
            and self.path[-1] == target
            and all(cell not in self.known_obstacles for cell in self.path)
        )

    def _replan(self, world: GridWorld, target: Cell, mission: bool) -> bool:
        blocked = next(
            (c for c in self.path if c in self.known_obstacles), None
        )
        self.state = AgentState.REPLAN
        new_path = self._plan(world, target)
        if new_path is None:
            self._log(
                "replan_failed",
                "no route to "
                + ("the goal" if mission else "home")
                + " exists on the known map"
                + (f"; obstacle at {blocked} blocks the old path" if blocked else ""),
            )
            if mission:
                return self._start_return_home(
                    world,
                    "no route to the goal remains; returning home "
                    "while the battery still allows it",
                )
            self.abort("no route home exists on the known map")
            return False
        self.path = new_path
        if blocked is not None:
            cause = f"planned path is blocked by perceived obstacle at {blocked}"
        else:
            cause = "no valid plan; planning a fresh route"
        self._log(
            "replan",
            f"{cause}; new route has {len(new_path) - 1} moves, "
            f"cost {path_cost(new_path):.1f}",
        )
        if mission:
            cost = path_cost(new_path)
            if self.battery < cost:
                return self._start_return_home(
                    world,
                    f"replanned route to the goal costs {cost:.1f} but "
                    f"only {self.battery:.1f} battery remains; "
                    f"returning home instead",
                )
            self._log(
                "evaluate_options",
                f"continue to goal: route cost {cost:.1f}, battery "
                f"{self.battery:.1f}, margin {self.battery - cost:.1f}; "
                f"safe return home is still possible; decision: continue",
            )
        self.state = AgentState.NAVIGATE if mission else AgentState.RETURN_HOME
        return True

    def _start_return_home(self, world: GridWorld, reason: str) -> bool:
        path = self._plan(world, self.home)
        if path is None:
            self.abort("must return home but no route home is known")
            return False
        self.path = path
        self.state = AgentState.RETURN_HOME
        self._log("return_home", reason)
        return True

    # ------------------------------------------------------------------
    # Act
    # ------------------------------------------------------------------

    def _follow_one_step(self, target: Cell, mission: bool) -> None:
        if len(self.path) < 2:
            self._arrive(target, mission)
            return
        nxt = self.path[1]
        cost = move_cost(self.position, nxt)
        if self.battery < cost:
            self.abort(
                f"battery depleted at {self.position}; cannot move to {nxt}"
            )
            return
        self.battery -= cost
        self.position = nxt
        self.path.pop(0)
        self.path_taken.append(nxt)
        self._log(
            "move",
            f"moved to {nxt} en route to "
            f"{'goal' if mission else 'home'}; battery {self.battery:.1f} "
            f"remaining",
        )
        if nxt == target:
            self._arrive(target, mission)

    def _arrive(self, target: Cell, mission: bool) -> None:
        if mission:
            self.reached_goal = True
            self._log("arrive", f"reached the goal at {target}; landing")
        else:
            self._log("arrive", f"back at home {target}; landing")
        self.state = AgentState.LAND

    def _land(self) -> None:
        if self.reached_goal:
            reason = "touchdown at the goal; mission complete"
        else:
            reason = "touchdown at home; mission ended without reaching the goal"
        self._log("land", reason)
        self.state = AgentState.DONE

    # ------------------------------------------------------------------

    def _log(self, action: str, reason: str) -> None:
        self.decision_log.append(
            DecisionEntry(self.step_count, self.state.value, action, reason)
        )
