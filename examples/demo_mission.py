"""Demo mission: the built-in wall-and-surprise scenario, via the API.

Run with:  python examples/demo_mission.py
"""

from drone_autonomy_sim import build_demo_scenario, run_mission


def main() -> None:
    world, start, goal, events, battery = build_demo_scenario()
    result = run_mission(world, start, goal, events, battery)

    print(f"Success: {result.success} in {result.steps} steps")
    print(f"Final battery: {result.final_battery:.1f} "
          f"at {result.end_position}")
    print("\nDecision log:")
    for entry in result.decision_log:
        print(f"  step {entry.step:>3} [{entry.state:<11}] "
              f"{entry.action}: {entry.reason}")


if __name__ == "__main__":
    main()
