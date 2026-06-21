"""Юнит-тесты планировщика A* и контроллера pure-pursuit (чистая логика, без железа)."""

import numpy as np
import pytest

from follow_me.navigation import build_traversable, plan_path, pursuit_command

FREE = 255
OBST = 0


def _open_grid(n=20):
    return np.full((n, n), FREE, dtype=np.uint8)


def test_path_in_open_grid():
    grid = _open_grid()
    path = plan_path(grid, (2, 2), (17, 17), radius_px=0)
    assert path is not None
    assert path[0] == (2, 2)
    assert path[-1] == (17, 17)


def test_no_path_when_goal_blocked():
    grid = _open_grid()
    grid[10, 10] = OBST
    path = plan_path(grid, (2, 2), (10, 10), radius_px=0)
    assert path is None


def test_routes_around_wall():
    grid = _open_grid(21)
    # вертикальная стена в колонке 10, строки 0..17; проход снизу (строки 18..20)
    grid[0:18, 10] = OBST
    # старт слева от стены (col9), цель справа (col11), та же строка
    path = plan_path(grid, (9, 2), (11, 2), radius_px=0)
    assert path is not None
    # чтобы пересечь колонку 10, путь обязан уйти вниз ниже стены (row >= 18)
    assert any(r >= 18 for (c, r) in path)


def test_unknown_is_blocked():
    grid = _open_grid()
    grid[:] = 127  # всё неизвестно
    path = plan_path(grid, (2, 2), (17, 17), radius_px=0)
    assert path is None


def test_inflation_blocks_near_obstacle():
    grid = _open_grid()
    grid[10, 10] = OBST
    trav0 = build_traversable(grid, radius_px=0)
    trav2 = build_traversable(grid, radius_px=2)
    assert trav0[10, 11]          # без раздува соседняя свободна
    assert not trav2[10, 11]      # с раздувом 2px — занята


def test_pursuit_goal_reached():
    left, right, _ = pursuit_command(0, 0, 0, [(100.0, 0.0)],
                                     lookahead_mm=600, goal_tol_mm=200,
                                     speed=160, turn_gain=2.0)
    assert (left, right) == (0, 0)


def test_pursuit_straight_ahead():
    # цель прямо по курсу (theta=0 смотрит вдоль +x), далеко -> едем вперёд почти прямо
    left, right, _ = pursuit_command(0, 0, 0, [(3000.0, 0.0)],
                                     lookahead_mm=600, goal_tol_mm=200,
                                     speed=160, turn_gain=2.0)
    assert left > 0 and right > 0
    assert abs(left - right) <= 2  # почти прямо


def test_pursuit_turns_left_for_left_target():
    # цель слева (theta=0, цель в +y) -> поворот влево: левый борт МЕДЛЕННЕЕ правого
    left, right, _ = pursuit_command(0, 0, 0, [(3000.0, 3000.0)],
                                     lookahead_mm=600, goal_tol_mm=200,
                                     speed=160, turn_gain=2.0)
    assert left < right


def test_pursuit_turns_right_for_right_target():
    left, right, _ = pursuit_command(0, 0, 0, [(3000.0, -3000.0)],
                                     lookahead_mm=600, goal_tol_mm=200,
                                     speed=160, turn_gain=2.0)
    assert left > right


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
