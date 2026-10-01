"""Mission runner: applies scripted events and steps the agent.

A mission is a world, a start, a goal, a battery size, and a list of
scripted ``ObstacleEvent``s. An event makes a new obstacle appear in the
true world at a given step. The agent does not know about the event in
advance: it only finds out when the obstacle enters its sensor radius,
which is what forces a genuine perceive-decide-replan cycle.
"""

from __future__ import annotations

from dataclasses import dataclass

from .agent import AgentState, DecisionEntry, DroneAgent
from .world import Cell, GridWorld


@dataclass(frozen=True)
class ObstacleEvent:
    """A new obstacle that appears in the world at a given step."""

    step: int
    cell: Cell


@dataclass
class MissionResult:
    """The auditable outcome of one mission."""

    success: bool
    path_taken: list[Cell]
    steps: int
    decision_log: list[DecisionEntry]
    final_battery: float
    final_state: AgentState
    end_position: Cell


def run_mission(
    world: GridWorld,
    start: Cell,
    goal: Cell,
    events: list[ObstacleEvent] = (),
    battery: float = 100.0,
    *,
    sensor_radius: float = 3.0,
    reserve_margin: float = 2.0,
    max_steps: int = 500,
) -> MissionResult:
    """Fly one mission and return its full result.

    ``battery`` is the starting charge in abstract energy units; each
    orthogonal move costs 1.0 and each diagonal move costs sqrt(2),
    matching the planner's cost model.
    """
    agent = DroneAgent(
        start=start,
        goal=goal,
        battery_capacity=battery,
        sensor_radius=sensor_radius,
        reserve_margin=reserve_margin,
    )
    events_by_step: dict[int, list[Cell]] = {}
    for event in events:
        events_by_step.setdefault(event.step, []).append(event.cell)

    while not agent.finished and agent.step_count < max_steps:
        for cell in events_by_step.get(agent.step_count, []):
            world.add_obstacle(cell)
        agent.step(world)
    if not agent.finished:
        agent.abort(f"mission exceeded the {max_steps} step limit")

    return MissionResult(
        success=agent.reached_goal and agent.state is AgentState.DONE,
        path_taken=list(agent.path_taken),
        steps=agent.step_count,
        decision_log=list(agent.decision_log),
        final_battery=agent.battery,
        final_state=agent.state,
        end_position=agent.position,
    )


def build_demo_scenario():
    """The built-in demo used by the CLI and the examples.

    A 14 by 9 world split by a wall at x = 9 with a single gap at y = 4.
    The drone flies the straight corridor at y = 4 toward the goal. At
    step 2 a surprise obstacle appears at (5, 4), directly on its path.
    The battery is sized for an out-and-back safety margin, so after the
    detour the agent must genuinely evaluate whether continuing is safe.
    """
    width, height = 14, 9
    wall_x, gap_y = 9, 4
    obstacles = {(wall_x, y) for y in range(height) if y != gap_y}
    world = GridWorld(width, height, obstacles)
    start = (0, 4)
    goal = (13, 4)
    events = [ObstacleEvent(step=2, cell=(5, 4))]
    battery = 45.0
    return world, start, goal, events, battery
