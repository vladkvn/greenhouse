"""Контроллер колёсного робота: едет вперёд и объезжает препятствия.

Использует два передних датчика расстояния для принятия решений,
а лидар и камеру — включает для визуализации и сбора данных.
Имена устройств должны совпадать с именами в файле мира my_robot.wbt.
"""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.0       # рад/с — крейсерская скорость колёс
THRESHOLD = 150.0     # порог датчика: выше => препятствие ближе ~0.85 м

# --- Моторы колёс (левая и правая стороны) ---
right_motors = [robot.getDevice("wheel1"), robot.getDevice("wheel3")]
left_motors = [robot.getDevice("wheel2"), robot.getDevice("wheel4")]
for m in left_motors + right_motors:
    m.setPosition(float("inf"))   # режим управления скоростью
    m.setVelocity(0.0)

# --- Датчики расстояния ---
ds_left = robot.getDevice("ds_left")
ds_right = robot.getDevice("ds_right")
ds_left.enable(timestep)
ds_right.enable(timestep)

# --- Лидар (данные расстояний по кругу + облако точек для 3D-вида) ---
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()

# --- Камера ---
camera = robot.getDevice("camera")
camera.enable(timestep)


def set_speed(left_speed, right_speed):
    for m in left_motors:
        m.setVelocity(left_speed)
    for m in right_motors:
        m.setVelocity(right_speed)


print("Контроллер запущен. Робот едет и объезжает препятствия.")

while robot.step(timestep) != -1:
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()

    if left_val > THRESHOLD or right_val > THRESHOLD:
        # Есть препятствие — поворачиваем в сторону от ближайшего
        if left_val > right_val:
            set_speed(MAX_SPEED, -MAX_SPEED)   # препятствие слева -> поворот вправо
        else:
            set_speed(-MAX_SPEED, MAX_SPEED)   # препятствие справа -> поворот влево
    else:
        # Путь свободен — едем прямо
        set_speed(MAX_SPEED, MAX_SPEED)

    # Пример доступа к данным лидара (можно использовать для своей логики):
    # ranges = lidar.getRangeImage()   # список расстояний длиной 360
    # min_front = min(ranges[170:190]) # минимальное расстояние спереди
