import pytest

from drone_autonomy_sim import GridWorld


def test_bounds_and_free_cells():
    world = GridWorld(5, 4)
    assert world.in_bounds((0, 0))
    assert world.in_bounds((4, 3))
    assert not world.in_bounds((5, 0))
    assert not world.in_bounds((0, -1))
    assert world.is_free((2, 2))
    assert not world.is_free((9, 9))


def test_add_and_remove_obstacle():
    world = GridWorld(5, 5)
    world.add_obstacle((2, 2))
    assert world.is_occupied((2, 2))
    assert not world.is_free((2, 2))
    world.remove_obstacle((2, 2))
    assert world.is_free((2, 2))


def test_obstacle_out_of_bounds_rejected():
    world = GridWorld(5, 5)
    with pytest.raises(ValueError):
        world.add_obstacle((5, 0))
    with pytest.raises(ValueError):
        GridWorld(5, 5, obstacles={(0, 7)})
