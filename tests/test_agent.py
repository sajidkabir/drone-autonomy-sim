from drone_autonomy_sim import (
    AgentState,
    DroneAgent,
    GridWorld,
    ObstacleEvent,
    run_mission,
)


def wall_world(width=12, height=7):
    obstacles = {(6, y) for y in range(height) if y != 3}
    return GridWorld(width, height, obstacles=obstacles)


def test_agent_reaches_goal_in_static_world():
    world = wall_world()
    result = run_mission(world, (0, 3), (11, 3), battery=100.0)
    assert result.success
    assert result.end_position == (11, 3)
    assert result.final_state is AgentState.DONE
    assert result.path_taken[0] == (0, 3)
    assert result.path_taken[-1] == (11, 3)


def test_dynamic_obstacle_triggers_replan_and_agent_arrives():
    # Open grid, straight corridor at y=2. An obstacle drops onto the
    # path at (5, 2) on step 1; the agent perceives it inside its sensor
    # radius, replans around it, and still reaches the goal.
    world = GridWorld(10, 5)
    events = [ObstacleEvent(step=1, cell=(5, 2))]
    result = run_mission(world, (0, 2), (9, 2), events, battery=100.0)
    assert result.success
    assert result.end_position == (9, 2)
    assert (5, 2) not in result.path_taken
    replans = [e for e in result.decision_log if e.state == "REPLAN"]
    assert replans, "expected at least one REPLAN decision"
    assert any(e.action == "replan" for e in replans)


def test_low_battery_returns_home_and_lands():
    # Open 20x3 strip, goal far away, battery far too small. Mid-flight
    # the safe-return guard must fire: RETURN_HOME is logged and the
    # drone lands back at its start instead of dying mid-grid.
    world = GridWorld(20, 3)
    result = run_mission(
        world, (0, 1), (19, 1), battery=14.0, reserve_margin=2.0
    )
    assert not result.success
    assert result.end_position == (0, 1)
    assert result.final_state is AgentState.DONE
    assert any(
        e.action == "return_home" for e in result.decision_log
    ), "expected a RETURN_HOME decision"
    assert result.final_battery >= 0.0


def test_unreachable_goal_agent_comes_home():
    # A full-height wall with no gap separates the agent from the goal.
    # Once the wall is perceived, no route exists: the agent must give up
    # on the goal and land back home rather than wander forever.
    obstacles = {(4, y) for y in range(5)}
    world = GridWorld(8, 5, obstacles=obstacles)
    result = run_mission(
        world, (0, 2), (7, 2), battery=200.0, sensor_radius=4.0
    )
    assert not result.success
    assert result.end_position == (0, 2)
    assert any(
        e.action == "replan_failed" for e in result.decision_log
    )


def test_decision_log_reasons_are_never_empty():
    from drone_autonomy_sim import build_demo_scenario

    world, start, goal, events, battery = build_demo_scenario()
    result = run_mission(world, start, goal, events, battery)
    assert result.decision_log
    steps = []
    for entry in result.decision_log:
        assert entry.action.strip()
        assert entry.reason.strip()
        steps.append(entry.step)
    assert steps == sorted(steps)


def test_battery_never_goes_negative():
    scenarios = []
    scenarios.append(
        run_mission(GridWorld(20, 3), (0, 1), (19, 1), battery=14.0)
    )
    scenarios.append(
        run_mission(
            GridWorld(10, 5), (0, 2), (9, 2),
            [ObstacleEvent(step=1, cell=(5, 2))], battery=100.0,
        )
    )
    scenarios.append(run_mission(wall_world(), (0, 3), (11, 3), battery=60.0))
    for result in scenarios:
        assert result.final_battery >= 0.0


def test_perception_is_limited_by_sensor_radius():
    world = GridWorld(20, 20, obstacles={(2, 1), (19, 19)})
    agent = DroneAgent(
        start=(0, 0), goal=(5, 0), battery_capacity=50.0, sensor_radius=3.0
    )
    agent.step(world)  # takeoff step: perceives from the start cell
    assert (2, 1) in agent.known_obstacles
    assert (19, 19) not in agent.known_obstacles
