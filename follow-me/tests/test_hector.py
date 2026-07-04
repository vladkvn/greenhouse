"""Юнит-тест Hector scan matcher на синтетике: matcher должен восстановить позу.

Строим синтетическую комнату-коробку, генерируем скан из ИЗВЕСТНОЙ позы лучекастингом,
сбиваем позу matcher'а, прогоняем _match и проверяем, что поза вернулась к истинной.
Это валидирует математику Gauss-Newton без железа.
"""

import math

import numpy as np
import pytest

from follow_me.config import FollowMeConfig
from follow_me.hector import HectorSlam


def _make_room(h: HectorSlam):
    """Коробка-комната: стены при 1 и 9 м, внутри свободно."""
    h.prob[:] = 0.1
    w = int(round(1.0 / h.res))
    W = int(round(9.0 / h.res))
    h.prob[w, w:W] = 1.0      # стена y=1
    h.prob[W, w:W] = 1.0      # стена y=9
    h.prob[w:W, w] = 1.0      # стена x=1
    h.prob[w:W, W] = 1.0      # стена x=9


def _raycast(h: HectorSlam, xt, yt, tht):
    """Скан (360 бин, м) из позы (xt,yt,tht) против карты h.prob."""
    scan = np.full(360, np.nan)
    res = h.res
    n = h.n
    maxr = int(12.0 / res)
    for a in range(360):
        phi = math.radians(a) * h._cfg.hector_scan_dir
        wdir = tht + phi
        dx, dy = math.cos(wdir), math.sin(wdir)
        for k in range(1, maxr):
            r = k * res
            cx = int((xt + r * dx) / res)
            cy = int((yt + r * dy) / res)
            if not (0 <= cx < n and 0 <= cy < n):
                break
            if h.prob[cy, cx] > 0.5:
                scan[a] = r
                break
    return scan


def test_hector_recovers_pose():
    cfg = FollowMeConfig()
    cfg.hector_match_theta = True
    h = HectorSlam(cfg)
    _make_room(h)
    xt, yt, tht = 5.0, 5.0, 0.3
    scan = _raycast(h, xt, yt, tht)
    sp = h._scan_points(scan)
    assert sp is not None
    pts, _ = sp
    # сбиваем позу
    h.x, h.y, h.theta = xt + 0.15, yt - 0.12, tht - 0.08
    h._inited = True
    h._match(pts)
    assert abs(h.x - xt) < 0.05, f"x={h.x}"
    assert abs(h.y - yt) < 0.05, f"y={h.y}"
    assert abs(h.theta - tht) < 0.03, f"theta={h.theta}"


def test_hector_no_move_when_aligned():
    cfg = FollowMeConfig()
    cfg.hector_match_theta = True
    h = HectorSlam(cfg)
    _make_room(h)
    xt, yt, tht = 4.5, 5.5, -0.2
    scan = _raycast(h, xt, yt, tht)
    pts, _ = h._scan_points(scan)
    h.x, h.y, h.theta = xt, yt, tht   # уже точно
    h._inited = True
    h._match(pts)
    assert abs(h.x - xt) < 0.02
    assert abs(h.y - yt) < 0.02
    assert abs(h.theta - tht) < 0.01


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
