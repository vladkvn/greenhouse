"""Построение карты: сессия SLAM и хранилище карт.

Оркестрация владеет жизненным циклом сессии (start/stop). Конкретный алгоритм
(evidence-сетка по лидару сейчас, slam_toolbox через адаптер позже) скрыт за Protocol.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Iterator
from pathlib import Path
from typing import Protocol

from pydantic import BaseModel

from greenhouse.domain.geometry import Point2D, Pose2D
from greenhouse.domain.grid import CellState, MapMeta, OccupancyGrid
from greenhouse.sensing.interfaces import LidarScan


class MapBuilder(Protocol):
    """Инкрементальное построение карты из сканов лидара.

    Поза передаётся снаружи (из локализации/одометрии), чтобы строитель карты не
    зависел жёстко от конкретного локализатора.
    """

    def begin_session(self) -> None: ...

    def ingest_scan(self, *, scan: LidarScan, pose_xytheta: tuple[float, float, float]) -> None:
        """Добавить наблюдение в карту с известной (оценённой) позой робота."""
        ...

    def current_map(self) -> OccupancyGrid:
        """Текущее наилучшее представление карты."""
        ...

    def end_session(self) -> OccupancyGrid:
        """Завершить сессию и вернуть итоговую карту."""
        ...


class EvidenceGridMapper:
    """Реализует `MapBuilder`: лог-odds сетка занятости по лидар-рейкасту.

    Инверсная модель датчика: вдоль каждого луча ячейки до точки возврата получают
    свидетельство «свободно», а ячейка попадания — «занято». Свидетельства копятся в
    лог-odds, поэтому карта устойчива к шуму — повторные наблюдения усиливают уверенность,
    одиночный выброс не переключает ячейку. Луч растеризуется алгоритмом Брезенхэма по той
    же сетке, что потом увидят планировщик и проверка коллизий (единая модель занятости).

    Поза приходит снаружи как `(x, y, theta)` в координатах карты; углы скана считаются
    относительно курса робота, поэтому мировой угол луча = `theta + angle_min + i*incr`.
    """

    def __init__(
        self,
        *,
        meta: MapMeta,
        l_free: float = 0.4,
        l_occ: float = 0.85,
        l_clamp: float = 5.0,
        occ_threshold: float = 0.5,
        free_threshold: float = -0.35,
    ) -> None:
        self._meta = meta
        self._l_free = l_free
        self._l_occ = l_occ
        self._l_clamp = l_clamp
        self._occ_threshold = occ_threshold
        self._free_threshold = free_threshold
        self._logodds = [0.0] * (meta.width_px * meta.height_px)

    @property
    def meta(self) -> MapMeta:
        return self._meta

    def begin_session(self) -> None:
        self._logodds = [0.0] * (self._meta.width_px * self._meta.height_px)

    def ingest_scan(self, *, scan: LidarScan, pose_xytheta: tuple[float, float, float]) -> None:
        x, y, theta = pose_xytheta
        origin = self._meta.world_to_cell(Point2D(x_m=x, y_m=y))
        for i, r in enumerate(scan.ranges_m):
            angle = theta + scan.angle_min_rad + i * scan.angle_increment_rad
            hit = math.isfinite(r) and r < scan.range_max_m
            reach = r if hit else scan.range_max_m
            end = self._meta.world_to_cell(
                Point2D(x_m=x + reach * math.cos(angle), y_m=y + reach * math.sin(angle))
            )
            cells = list(_bresenham(origin, end))
            # Все ячейки до конечной — свободны; конечная — занята при реальном возврате.
            for cell in cells[:-1]:
                self._update(cell, -self._l_free)
            self._update(cells[-1], self._l_occ if hit else -self._l_free)

    def current_map(self) -> OccupancyGrid:
        return OccupancyGrid(meta=self._meta, cells=[self._classify(lo) for lo in self._logodds])

    def end_session(self) -> OccupancyGrid:
        return self.current_map()

    def _update(self, cell: tuple[int, int], delta: float) -> None:
        row, col = cell
        if not self._meta.in_bounds(row, col):
            return
        idx = row * self._meta.width_px + col
        self._logodds[idx] = _clamp(self._logodds[idx] + delta, -self._l_clamp, self._l_clamp)

    def _classify(self, logodds: float) -> CellState:
        if logodds > self._occ_threshold:
            return CellState.OCCUPIED
        if logodds < self._free_threshold:
            return CellState.FREE
        return CellState.UNKNOWN


_FRONTIER_NEIGHBORS = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))


def nearest_frontier(
    grid: OccupancyGrid,
    start: tuple[int, int],
    *,
    min_cells: int = 1,
    clear_rad: int = 0,
    blacklist: set[tuple[int, int]] | None = None,
) -> tuple[int, int] | None:
    """Ближайшая ОТКРЫТАЯ граница «свободно↔неизвестно» (для frontier-исследования).

    BFS по известному свободному пространству от `start`; возвращает свободную ячейку,
    которая граничит с `UNKNOWN`, отстоит от старта не ближе `min_cells`, имеет запас
    `clear_rad` от занятых ячеек (пристенные ложные границы за стеной пропускаем) и не лежит
    в окрестности (`min_cells`) ранее недостижимых границ из `blacklist`. None — границ нет
    (достижимое пространство разведано)."""
    black = blacklist or set()
    meta = grid.meta
    queue: deque[tuple[int, int]] = deque([start])
    seen = {start}
    while queue:
        r, c = queue.popleft()
        if (
            grid.at(r, c) is CellState.FREE
            and abs(r - start[0]) + abs(c - start[1]) >= min_cells
            and _has_unknown_neighbor(grid, r, c)
            and not _near_occupied(grid, r, c, clear_rad)
            and not any(abs(r - br) <= min_cells and abs(c - bc) <= min_cells for br, bc in black)
        ):
            return r, c
        for dr, dc in _FRONTIER_NEIGHBORS:
            nb = (r + dr, c + dc)
            if nb not in seen and meta.in_bounds(nb[0], nb[1]) and grid.at(*nb) is CellState.FREE:
                seen.add(nb)
                queue.append(nb)
    return None


def _has_unknown_neighbor(grid: OccupancyGrid, row: int, col: int) -> bool:
    return any(grid.at(row + dr, col + dc) is CellState.UNKNOWN for dr, dc in _FRONTIER_NEIGHBORS)


def _near_occupied(grid: OccupancyGrid, row: int, col: int, rad: int) -> bool:
    return any(
        grid.at(row + dr, col + dc) is CellState.OCCUPIED
        for dr in range(-rad, rad + 1)
        for dc in range(-rad, rad + 1)
    )


def _bresenham(start: tuple[int, int], end: tuple[int, int]) -> Iterator[tuple[int, int]]:
    """Целочисленные ячейки `(row, col)` вдоль отрезка start→end включительно."""
    r0, c0 = start
    r1, c1 = end
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        yield r, c
        if r == r1 and c == c1:
            return
        e2 = 2 * err
        if e2 >= -dr:
            err -= dr
            c += sc
        if e2 <= dc:
            err += dc
            r += sr


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class StoredMap(BaseModel):
    label: str
    grid: OccupancyGrid


class MapStore(Protocol):
    """Загрузка/сохранение карт по семантическим меткам (например 'greenhouse_main')."""

    def save(self, *, label: str, grid: OccupancyGrid) -> None: ...

    def load(self, *, label: str) -> StoredMap | None: ...

    def list_labels(self) -> list[str]: ...


# --- Непрерывная актуализация карты (Инкремент 11) ---


class ConfidenceGatedMapper:
    """Обёртка над `MapBuilder`: поглощает скан только при достаточной уверенности позы.

    Защита от «размазывания» карты: если локализация неуверенна (низкий `confidence`),
    наблюдение игнорируется — плохая поза не портит стабильную карту. Иначе скан штатно
    подмешивается, поэтому появившиеся/исчезнувшие препятствия отражаются при проездах
    (лог-odds сам усиливает занятость и очищает её при повторных свободных наблюдениях).
    """

    def __init__(self, *, mapper: MapBuilder, min_confidence: float = 0.5) -> None:
        self._mapper = mapper
        self._min_conf = min_confidence

    def begin_session(self) -> None:
        self._mapper.begin_session()

    def maybe_ingest(self, *, scan: LidarScan, pose: Pose2D, confidence: float) -> bool:
        """Подмешать скан, если уверенность достаточна. True — приняли, False — пропустили."""
        if confidence < self._min_conf:
            return False
        self._mapper.ingest_scan(
            scan=scan, pose_xytheta=(pose.x_m, pose.y_m, pose.theta_rad)
        )
        return True

    def current_map(self) -> OccupancyGrid:
        return self._mapper.current_map()


class DynamicObstacleLayer:
    """Краткоживущий слой динамических препятствий поверх стабильной карты.

    Ячейка, где лидар видит препятствие, которого нет в стабильной карте, помечается с TTL;
    со временем «выветривается» (`decay`). Так внезапная помеха учитывается планировщиком
    сразу, а после исчезновения слой сам очищается — стабильная карта при этом не трогается.
    """

    def __init__(self, *, meta: MapMeta, ttl_ticks: int = 30) -> None:
        self._meta = meta
        self._ttl = ttl_ticks
        self._age: dict[tuple[int, int], int] = {}

    def observe(self, *, scan: LidarScan, pose: Pose2D, base: OccupancyGrid) -> None:
        meta = self._meta
        for i, r in enumerate(scan.ranges_m):
            if not math.isfinite(r) or r >= scan.range_max_m:
                continue
            angle = pose.theta_rad + scan.angle_min_rad + i * scan.angle_increment_rad
            hit = Point2D(x_m=pose.x_m + r * math.cos(angle), y_m=pose.y_m + r * math.sin(angle))
            row, col = meta.world_to_cell(hit)
            if meta.in_bounds(row, col) and base.at(row, col) is not CellState.OCCUPIED:
                self._age[(row, col)] = self._ttl  # помеха вне стабильной карты — динамика

    def decay(self) -> None:
        for cell in list(self._age):
            self._age[cell] -= 1
            if self._age[cell] <= 0:
                del self._age[cell]

    def cells(self) -> set[tuple[int, int]]:
        return set(self._age)

    def apply_to(self, grid: OccupancyGrid) -> OccupancyGrid:
        """Наложить динамические препятствия на карту (для планирования)."""
        if not self._age:
            return grid
        cells = list(grid.cells)
        w = grid.meta.width_px
        for row, col in self._age:
            if grid.meta.in_bounds(row, col):
                cells[row * w + col] = CellState.OCCUPIED
        return OccupancyGrid(meta=grid.meta, cells=cells)


def merge_occupancy(base: OccupancyGrid, update: OccupancyGrid) -> OccupancyGrid:
    """Слить карты: где `update` знает (FREE/OCCUPIED) — берём его, иначе оставляем `base`."""
    if base.meta != update.meta:
        raise ValueError("карты несовместимы: разные meta")
    cells = [
        u if u is not CellState.UNKNOWN else b
        for b, u in zip(base.cells, update.cells, strict=True)
    ]
    return OccupancyGrid(meta=base.meta, cells=cells)


class InMemoryMapStore:
    """Реализует `MapStore` в памяти (живёт в пределах процесса)."""

    def __init__(self) -> None:
        self._maps: dict[str, StoredMap] = {}

    def save(self, *, label: str, grid: OccupancyGrid) -> None:
        self._maps[label] = StoredMap(label=label, grid=grid)

    def load(self, *, label: str) -> StoredMap | None:
        return self._maps.get(label)

    def list_labels(self) -> list[str]:
        return sorted(self._maps)


class FileMapStore:
    """Реализует `MapStore` поверх файлов JSON — карта переживает перезапуск процесса."""

    def __init__(self, *, directory: str | Path) -> None:
        self._dir = Path(directory)
        self._dir.mkdir(parents=True, exist_ok=True)

    def _path(self, label: str) -> Path:
        return self._dir / f"{label}.json"

    def save(self, *, label: str, grid: OccupancyGrid) -> None:
        self._path(label).write_text(
            StoredMap(label=label, grid=grid).model_dump_json(), encoding="utf-8"
        )

    def load(self, *, label: str) -> StoredMap | None:
        path = self._path(label)
        if not path.exists():
            return None
        return StoredMap.model_validate_json(path.read_text(encoding="utf-8"))

    def list_labels(self) -> list[str]:
        return sorted(p.stem for p in self._dir.glob("*.json"))
