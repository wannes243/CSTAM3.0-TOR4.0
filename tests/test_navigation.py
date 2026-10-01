from navigation import OccupancyGrid, NavigationController


def free_grid():
    grid = OccupancyGrid(1.0, 5.0)
    grid.data[:, :] = 0
    return grid


def test_goal_controller_produces_command_and_avoids_inflated_wall():
    grid = free_grid()
    # A wall at x=0 with a gap at y=2; the planner must use the gap.
    gx = grid.cell(0, 0)[0]
    for y in range(grid.size):
        if y != grid.cell(0, 2)[1]:
            grid.data[y, gx] = 100

    controller = NavigationController(grid, safety_margin_m=0.0, max_linear_mps=0.3)
    controller.set_goal(3.0, 0.0)
    command = controller.command((-3.0, 0.0, 0.0))

    assert controller.status == "following"
    assert command[0] >= 0.0
    assert controller.path
    assert all(grid.data[y, x] != 100 for x, y in controller.path)


def test_exploration_stops_when_no_frontier_exists():
    grid = free_grid()
    controller = NavigationController(grid)
    controller.explore()

    # A completely known map has no frontier.
    controller.command((0.0, 0.0, 0.0))
    assert controller.status == "exploration_complete"


def test_goal_path_replans_when_new_obstacle_blocks_remaining_route():
    grid = free_grid()
    controller = NavigationController(grid, safety_margin_m=0.0)
    controller.set_goal(3.0, 0.0)
    controller.command((-3.0, 0.0, 0.0))
    assert controller.path

    blocked_cell = controller.path[min(3, len(controller.path) - 1)]
    blocked_point = grid.point(blocked_cell)
    grid.set_dynamic_occupied(*blocked_point)

    controller.command((-3.0, 0.0, 0.0))
    assert controller.status in {"following", "replanning"}
    assert blocked_cell not in controller.path
