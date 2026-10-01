import math

from drone_autonomy_sim import GridWorld, find_path, path_cost


def test_astar_open_grid_diagonal_shortest():
    # In an open 5x5 grid the optimum from corner to corner is four
    # diagonal moves: 5 cells, cost 4 * sqrt(2).
    world = GridWorld(5, 5)
    path = find_path(world, (0, 0), (4, 4))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)
    assert len(path) == 5
    assert math.isclose(path_cost(path), 4 * math.sqrt(2), rel_tol=1e-9)


def test_astar_returns_none_when_goal_walled_off():
    world = GridWorld(5, 5, obstacles={(3, 4), (4, 3), (3, 3)})
    assert find_path(world, (0, 0), (4, 4)) is None


def test_astar_no_corner_cutting():
    # Both orthogonal neighbors of the diagonal are blocked, so the
    # diagonal step (0,0) -> (1,1) is illegal and no route exists.
    world = GridWorld(2, 2, obstacles={(1, 0), (0, 1)})
    assert find_path(world, (0, 0), (1, 1)) is None
    # With only one side blocked the planner must walk around the corner.
    world = GridWorld(3, 3, obstacles={(1, 0)})
    path = find_path(world, (0, 0), (1, 1))
    assert path is not None
    assert path[1] == (0, 1)
    assert math.isclose(path_cost(path), 2.0, rel_tol=1e-9)


def test_astar_path_is_valid_on_wall_grid():
    obstacles = {(4, y) for y in range(7) if y != 3}
    world = GridWorld(9, 7, obstacles=obstacles)
    path = find_path(world, (0, 3), (8, 3))
    assert path is not None
    assert path[0] == (0, 3) and path[-1] == (8, 3)
    for cell in path:
        assert world.is_free(cell)
    for a, b in zip(path, path[1:]):
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1
    # The only way through is the gap at (4, 3).
    assert (4, 3) in path
