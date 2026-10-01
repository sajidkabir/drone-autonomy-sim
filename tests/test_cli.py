from drone_autonomy_sim.cli import main


def test_cli_demo_runs_and_succeeds(capsys):
    exit_code = main(["run"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "Mission result" in out
    assert "SUCCESS, goal reached" in out
    assert "Decision log" in out
    assert "REPLAN" in out
    assert "return_home" not in out


def test_cli_low_battery_demo_comes_home(capsys):
    exit_code = main(["run", "--battery", "20"])
    out = capsys.readouterr().out
    assert exit_code == 1
    assert "GOAL NOT REACHED" in out
    assert "return_home" in out
