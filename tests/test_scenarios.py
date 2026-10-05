"""Tests for the v1.1 scenario library."""

import pytest

from drone_autonomy_sim.agent import AgentState
from drone_autonomy_sim.cli import main
from drone_autonomy_sim.scenarios import (
    get_scenario,
    scenario_names,
    world_from_ascii,
)
from drone_autonomy_sim.simulation import run_mission

_SMALL_MAP = """\
S#.
...
.G#\
"""


def test_world_from_ascii_parses_map():
    world, start, goal = world_from_ascii(_SMALL_MAP)
    assert (world.width, world.height) == (3, 3)
    assert start == (0, 2)
    assert goal == (1, 0)
    assert world.is_occupied((1, 2))
    assert world.is_occupied((2, 0))
    assert world.is_free((0, 0))


def test_world_from_ascii_rejects_bad_maps():
    with pytest.raises(ValueError):
        world_from_ascii("")
    with pytest.raises(ValueError):
        world_from_ascii("S.\n...\n.G")  # ragged rows
    with pytest.raises(ValueError):
        world_from_ascii("SX\n.G")  # unknown character
    with pytest.raises(ValueError):
        world_from_ascii("..\n.G")  # no start
    with pytest.raises(ValueError):
        world_from_ascii("S.\n..")  # no goal
    with pytest.raises(ValueError):
        world_from_ascii("SS\n.G")  # two starts


def test_scenario_registry_lists_four():
    assert scenario_names() == ["demo", "gauntlet", "long_haul", "clutter"]


def test_get_scenario_unknown_raises():
    with pytest.raises(KeyError):
        get_scenario("does-not-exist")


@pytest.mark.parametrize("name", scenario_names())
def test_scenarios_have_valid_start_goal_and_events(name):
    scenario = get_scenario(name)
    assert scenario.world.is_free(scenario.start)
    assert scenario.world.is_free(scenario.goal)
    assert scenario.start != scenario.goal
    for event in scenario.events:
        assert scenario.world.is_free(event.cell), (
            f"scenario {name}: event cell {event.cell} is not free"
        )


def _fly(name, **kwargs):
    scenario = get_scenario(name)
    return run_mission(
        scenario.world.copy(),
        scenario.start,
        scenario.goal,
        list(scenario.events),
        scenario.battery,
        sensor_radius=scenario.sensor_radius,
        reserve_margin=scenario.reserve_margin,
        max_steps=2000,
        **kwargs,
    )


def test_demo_still_succeeds():
    result = _fly("demo")
    assert result.success
    assert result.final_state is AgentState.DONE
    assert result.end_position == (13, 4)


def test_gauntlet_threads_three_walls():
    result = _fly("gauntlet")
    assert result.success
    assert result.final_state is AgentState.DONE
    assert result.steps == 67
    assert result.end_position == (32, 10)
    replans = [
        e for e in result.decision_log if e.action == "replan"
    ]
    senses = [e for e in result.decision_log if e.action == "sense"]
    assert any("(7, 11)" in e.reason for e in senses), (
        "the surprise obstacle at (7, 11) must be perceived mid-flight"
    )
    # The surprise genuinely changes the mission: without it the same
    # world is flown in 50 steps, with it in 67.
    scenario = get_scenario("gauntlet")
    no_event = run_mission(
        scenario.world.copy(),
        scenario.start,
        scenario.goal,
        [],
        scenario.battery,
        max_steps=2000,
    )
    assert no_event.success and no_event.steps == 50
    assert result.steps == 67 != no_event.steps


def test_long_haul_returns_home_safely():
    result = _fly("long_haul")
    assert not result.success
    assert result.final_state is AgentState.DONE
    assert result.end_position == (1, 6)
    assert result.final_battery > 0
    actions = [e.action for e in result.decision_log]
    assert "return_home" in actions


def test_clutter_succeeds_through_the_field():
    result = _fly("clutter")
    assert result.success
    assert result.final_state is AgentState.DONE
    assert result.steps == 39
    assert result.end_position == (38, 12)
    replans = [e for e in result.decision_log if e.action == "replan"]
    assert len(replans) >= 2


@pytest.mark.parametrize("name", scenario_names())
def test_scenarios_are_deterministic(name):
    first = _fly(name)
    second = _fly(name)
    assert (first.steps, first.end_position, first.final_battery) == (
        second.steps,
        second.end_position,
        second.final_battery,
    )


@pytest.mark.parametrize("name", scenario_names())
def test_scenarios_terminate_with_nonempty_log(name):
    result = _fly(name)
    assert result.final_state in (AgentState.DONE, AgentState.FAILED)
    assert len(result.decision_log) > 0


def test_cli_scenarios_lists_all(capsys):
    assert main(["scenarios"]) == 0
    out = capsys.readouterr().out
    for name in scenario_names():
        assert name in out


def test_cli_run_scenario_flag(capsys):
    exit_code = main(["run", "--scenario", "clutter"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "Scattered obstacle field" in out
    assert "SUCCESS, goal reached" in out


def test_cli_run_long_haul_comes_home(capsys):
    exit_code = main(["run", "--scenario", "long_haul"])
    out = capsys.readouterr().out
    assert exit_code == 1
    assert "GOAL NOT REACHED" in out
    assert "return_home" in out


def test_cli_run_unknown_scenario_exits_2():
    with pytest.raises(SystemExit) as exc:
        main(["run", "--scenario", "nope"])
    assert exc.value.code == 2
