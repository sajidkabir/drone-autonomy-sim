"""drone-autonomy-sim: onboard autonomous decision-making for drones.

A 2D autonomy simulation: a drone that perceives its surroundings within
a sensor radius, plans with A*, replans when the world changes under it,
and abandons its mission to return home when the battery no longer covers
a safe return. Every decision is logged with its reason.
"""

from .agent import AgentState, DecisionEntry, DroneAgent
from .planning import find_path, path_cost
from .simulation import (
    MissionResult,
    ObstacleEvent,
    build_demo_scenario,
    run_mission,
)
from .world import Cell, GridWorld

__version__ = "1.0.0"

__all__ = [
    "AgentState",
    "Cell",
    "DecisionEntry",
    "DroneAgent",
    "GridWorld",
    "MissionResult",
    "ObstacleEvent",
    "build_demo_scenario",
    "find_path",
    "path_cost",
    "run_mission",
    "__version__",
]
