"""Исследование теплицы по фронтиру (frontier-based exploration) + построение карты.

Робот строит карту занятости по лидару и едет НЕ хаотично, а целенаправленно:
- находит "фронтир" — границу между изученным (свободным) и неизвестным;
- прокладывает к ближайшему фронтиру маршрут по карте (BFS) с объездом грядок;
- доезжает и повторяет, пока неизвестных зон не останется;
- когда фронтиров больше нет — карта построена, робот останавливается.

Поза берётся из GPS + InertialUnit (в симуляции — точная). Это всё ещё
"mapping with known poses"; полный SLAM с оценкой позы — это ROS 2 + slam_toolbox.

Карта рисуется на узле Display и сохраняется в map.pgm рядом с контроллером.
"""

import math
from collections import deque
from controller import Robot, Display

robot = Robot()
timestep = int(robot.getBasicTimeStep())


def get_device(name):
    d = robot.getDevice(name)
    if d is None:
        print("ВНИМАНИЕ: устройство '%s' не найдено!" % name)
    return d


# ----------------- ПРИВОД -----------------
MAX_SPEED = 6.0
CRUISE = 4.0          # крейсерская скорость вперёд
TURN = 4.0           # скорость поворота на месте
WHEEL_R = 0.04       # радиус колеса (м) — для оценки скорости
EMERG = 700.0        # порог датчиков для аварийного объезда

right_motors = [get_device("wheel1"), get_device("wheel3")]
left_motors = [get_device("wheel2"), get_device("wheel4")]
for m in left_motors + right_motors:
    if m:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

ds_left = get_device("ds_left")
ds_right = get_device("ds_right")
if ds_left:
    ds_left.enable(timestep)
if ds_right:
    ds_right.enable(timestep)


def set_speed(left_speed, right_speed):
    for m in left_motors:
        if m:
            m.setVelocity(left_speed)
    for m in right_motors:
        if m:
            m.setVelocity(right_speed)


# ----------------- СЕНСОРЫ -----------------
lidar = get_device("lidar")
gps = get_device("gps")
imu = get_device("imu")
display = get_device("map_display")

if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()
if gps:
    gps.enable(timestep)
if imu:
    imu.enable(timestep)

MAPPING_OK = all([lidar, gps, imu])
if MAPPING_OK:
    LIDAR_RES = lidar.getHorizontalResolution()
    LIDAR_FOV = lidar.getFov()
    LIDAR_MAXRANGE = lidar.getMaxRange()
    ANGLE_START = LIDAR_FOV / 2.0
    ANGLE_STEP = -LIDAR_FOV / LIDAR_RES
    RAY_SKIP = 2
    print("Маппинг включён: лидар %d лучей, дальность %.1f м" % (LIDAR_RES, LIDAR_MAXRANGE))
else:
    print("Нет лидара/GPS/IMU — исследование невозможно.")

# ----------------- КАРТА -----------------
GRID = 220
RES = 0.03
ORIGIN = -3.3
L_OCC = 0.7
L_FREE = -0.3
L_CLAMP = 6.0
grid = [[0.0] * GRID for _ in range(GRID)]

OCC_T = 0.5           # порог "занято"
FREE_T = -0.5         # порог "свободно"
INFLATE = 4           # инфляция препятствий (клеток) под габарит робота


def world_to_cell(x, y):
    return int((x - ORIGIN) / RES), int((y - ORIGIN) / RES)


def cell_to_world(cx, cy):
    return ORIGIN + (cx + 0.5) * RES, ORIGIN + (cy + 0.5) * RES


def in_bounds(cx, cy):
    return 0 <= cx < GRID and 0 <= cy < GRID


def update_cell(cx, cy, delta):
    if in_bounds(cx, cy):
        v = grid[cy][cx] + delta
        grid[cy][cx] = max(-L_CLAMP, min(L_CLAMP, v))


def is_free(cx, cy):
    return grid[cy][cx] < FREE_T


def is_unknown(cx, cy):
    return FREE_T <= grid[cy][cx] <= OCC_T


def bresenham(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    guard = 0
    while not (x == x1 and y == y1) and guard < 4 * GRID:
        cells.append((x, y))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        guard += 1
    return cells


# ----------------- ПЛАНИРОВАНИЕ -----------------
def build_blocked():
    """2D-маска проходимости: True = ехать нельзя (занято + инфляция)."""
    blocked = [[False] * GRID for _ in range(GRID)]
    for cy in range(GRID):
        row = grid[cy]
        for cx in range(GRID):
            if row[cx] > OCC_T:
                for dy in range(-INFLATE, INFLATE + 1):
                    ny = cy + dy
                    if 0 <= ny < GRID:
                        brow = blocked[ny]
                        for dx in range(-INFLATE, INFLATE + 1):
                            nx = cx + dx
                            if 0 <= nx < GRID:
                                brow[nx] = True
    return blocked


def plan_to_frontier(start_cx, start_cy, blocked):
    """BFS от робота по свободным клеткам до ближайшего фронтира.
    Возвращает список клеток пути [старт..цель] или None."""
    if not in_bounds(start_cx, start_cy):
        return None
    visited = [[False] * GRID for _ in range(GRID)]
    parent = {}
    q = deque()
    q.append((start_cx, start_cy))
    visited[start_cy][start_cx] = True
    goal = None
    MIN_DIST_CELLS = 6
    parent[(start_cx, start_cy)] = None
    # глубина для отсечки слишком близких целей
    depth = {(start_cx, start_cy): 0}
    while q:
        cx, cy = q.popleft()
        d = depth[(cx, cy)]
        # фронтир: свободная клетка рядом с неизвестной
        if d >= MIN_DIST_CELLS:
            is_frontier = False
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if in_bounds(nx, ny) and is_unknown(nx, ny):
                    is_frontier = True
                    break
            if is_frontier:
                goal = (cx, cy)
                break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if not in_bounds(nx, ny) or visited[ny][nx]:
                continue
            if blocked[ny][nx] or not is_free(nx, ny):
                continue
            visited[ny][nx] = True
            parent[(nx, ny)] = (cx, cy)
            depth[(nx, ny)] = d + 1
            q.append((nx, ny))
    if goal is None:
        return None
    # восстановить путь
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


# ----------------- ОТРИСОВКА / СОХРАНЕНИЕ -----------------
def render_map(rcx, rcy, path):
    if display is None:
        return
    buf = bytearray(GRID * GRID * 4)
    for cy in range(GRID):
        base = (GRID - 1 - cy) * GRID
        row = grid[cy]
        for cx in range(GRID):
            lo = row[cx]
            c = 0 if lo > OCC_T else (255 if lo < FREE_T else 128)
            idx = (base + cx) * 4
            buf[idx] = buf[idx + 1] = buf[idx + 2] = c
            buf[idx + 3] = 255
    # путь — синим
    if path:
        for (cx, cy) in path:
            if in_bounds(cx, cy):
                idx = ((GRID - 1 - cy) * GRID + cx) * 4
                buf[idx] = 60
                buf[idx + 1] = 120
                buf[idx + 2] = 255
    # робот — красным
    if in_bounds(rcx, rcy):
        idx = ((GRID - 1 - rcy) * GRID + rcx) * 4
        buf[idx] = 255
        buf[idx + 1] = 0
        buf[idx + 2] = 0
    img = display.imageNew(bytes(buf), Display.RGBA, GRID, GRID)
    display.imagePaste(img, 0, 0, False)
    display.imageDelete(img)


def save_pgm(filename="map.pgm"):
    with open(filename, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (GRID, GRID)).encode("ascii"))
        rowbuf = bytearray(GRID)
        for cy in range(GRID - 1, -1, -1):
            row = grid[cy]
            for cx in range(GRID):
                lo = row[cx]
                rowbuf[cx] = 0 if lo > OCC_T else (255 if lo < FREE_T else 128)
            f.write(bytes(rowbuf))


def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def count_known():
    n = 0
    for row in grid:
        for lo in row:
            if lo > OCC_T or lo < FREE_T:
                n += 1
    return n


# ----------------- ГЛАВНЫЙ ЦИКЛ -----------------
print("Старт исследования теплицы.")
step_count = 0
warned = False
DONE = False

path = []
path_idx = 0
no_plan_tries = 0      # сколько раз подряд не нашли маршрут
recover = 0            # шаги манёвра восстановления
recover_dir = 1
anchor_x = None        # позиция для детектора застревания
anchor_y = None
anchor_step = 0

PLAN_EVERY = 50        # перепланирование (шагов)
REACH_CELLS = 4        # дошли до точки пути, если ближе стольких клеток
VERY_CLOSE = 950.0     # датчик: столкновение вот-вот (только как страховка)
STUCK_WIN = max(1, int(3.0 * 1000 / timestep))   # окно детектора застревания
STUCK_DIST = 0.06      # сдвинулся меньше -> застрял

# Завершение по "плато": если карта давно не прирастает — стоп.
known_prev = 0
stable_steps = 0
CHECK_EVERY = 30
GROWTH_MIN = 10
STABLE_LIMIT = max(1, int(18.0 * 1000 / timestep))   # ~18 c без прироста

while robot.step(timestep) != -1:
    step_count += 1

    if not MAPPING_OK:
        continue

    # --- поза ---
    pos = gps.getValues()
    rx, ry = pos[0], pos[1]
    yaw = imu.getRollPitchYaw()[2]
    rcx, rcy = world_to_cell(rx, ry)

    # --- маппинг (прорежен и защищён) ---
    if step_count % 3 == 0:
        try:
            ranges = lidar.getRangeImage()
            if ranges:
                for i in range(0, LIDAR_RES, RAY_SKIP):
                    r = ranges[i]
                    angle = yaw + ANGLE_START + i * ANGLE_STEP
                    hit = math.isfinite(r) and r < LIDAR_MAXRANGE
                    dist = r if hit else LIDAR_MAXRANGE
                    ex = rx + dist * math.cos(angle)
                    ey = ry + dist * math.sin(angle)
                    ecx, ecy = world_to_cell(ex, ey)
                    for (fx, fy) in bresenham(rcx, rcy, ecx, ecy):
                        update_cell(fx, fy, L_FREE)
                    if hit:
                        update_cell(ecx, ecy, L_OCC)
        except Exception as e:
            if not warned:
                print("Ошибка в маппинге:", e)
                warned = True

    # --- завершение по плато: карта перестала прирастать ---
    if not DONE and step_count % CHECK_EVERY == 0:
        known = count_known()
        if known - known_prev < GROWTH_MIN:
            stable_steps += CHECK_EVERY
        else:
            stable_steps = 0
        known_prev = known
        if stable_steps >= STABLE_LIMIT:
            DONE = True
            set_speed(0.0, 0.0)
            save_pgm()
            render_map(rcx, rcy, None)
            print("Карта построена (новых зон не появляется) — робот остановлен.")

    if DONE:
        set_speed(0.0, 0.0)
        if step_count % 24 == 0:
            render_map(rcx, rcy, None)
        continue

    lval = ds_left.getValue() if ds_left else 0.0
    rval = ds_right.getValue() if ds_right else 0.0

    # --- манёвр восстановления (когда застряли) ---
    if recover > 0:
        if recover_dir > 0:
            set_speed(-TURN, TURN)      # разворот на месте
        else:
            set_speed(TURN, -TURN)
        recover -= 1
        if recover == 0:
            path = []                   # форс-реплан после разворота
            path_idx = 0
            anchor_x, anchor_y, anchor_step = rx, ry, step_count
        continue

    # --- детектор застревания: давно не двигались -> развернуться ---
    if anchor_x is None:
        anchor_x, anchor_y, anchor_step = rx, ry, step_count
    if step_count - anchor_step >= STUCK_WIN:
        if math.hypot(rx - anchor_x, ry - anchor_y) < STUCK_DIST:
            recover = max(1, int(0.7 * 1000 / timestep))
            recover_dir = 1 if lval >= rval else -1   # в более свободную сторону
            continue
        anchor_x, anchor_y, anchor_step = rx, ry, step_count

    # --- перепланирование маршрута ---
    if step_count % PLAN_EVERY == 0 or path_idx >= len(path):
        try:
            blocked = build_blocked()
            new_path = plan_to_frontier(rcx, rcy, blocked)
        except Exception as e:
            new_path = None
            if not warned:
                print("Ошибка планирования:", e)
                warned = True
        if new_path and len(new_path) > 1:
            path = new_path
            path_idx = 1
            no_plan_tries = 0
        else:
            no_plan_tries += 1
            # несколько неудач подряд -> неизвестных зон не осталось
            if no_plan_tries >= 3:
                DONE = True
                set_speed(0.0, 0.0)
                save_pgm()
                render_map(rcx, rcy, None)
                print("Карта построена — все проходы исследованы. Сохранено в map.pgm")
                continue

    # --- движение к текущей точке пути ---
    if path and path_idx < len(path):
        tx, ty = cell_to_world(*path[path_idx])
        # дошли до точки -> следующая
        while path_idx < len(path) and math.hypot(tx - rx, ty - ry) < REACH_CELLS * RES:
            path_idx += 1
            if path_idx < len(path):
                tx, ty = cell_to_world(*path[path_idx])
        if path_idx >= len(path):
            set_speed(0.0, 0.0)
        else:
            desired = math.atan2(ty - ry, tx - rx)
            err = normalize(desired - yaw)
            if abs(err) > 0.4:
                # развернуться на месте в сторону цели
                if err > 0:
                    set_speed(-TURN, TURN)     # влево (CCW)
                else:
                    set_speed(TURN, -TURN)     # вправо (CW)
            elif lval > VERY_CLOSE or rval > VERY_CLOSE:
                # страховка: впереди вплотную -> развернуться и переплан
                recover = max(1, int(0.6 * 1000 / timestep))
                recover_dir = 1 if lval >= rval else -1
            else:
                # ехать вперёд с подруливанием
                steer = max(-2.0, min(2.0, 3.0 * err))
                set_speed(CRUISE - steer, CRUISE + steer)
    else:
        set_speed(0.0, 0.0)

    # --- отрисовка/сохранение ---
    if step_count % 24 == 0:
        render_map(rcx, rcy, path)
    if step_count % 400 == 0:
        save_pgm()
