#!/usr/bin/env bash
# Подъём ROS-стека ровера ОДНОЙ командой (запуск на Jetson).
# Предусловие: followme остановлен/disabled (иначе держит лидар):
#     sudo systemctl stop followme      # или разом навсегда: sudo systemctl disable --now followme
# Запуск:  bash ~/ros2_ws/scripts/rover_up.sh
# БЕЗ set -u: ROS setup.bash использует unbound-переменные (AMENT_TRACE_SETUP_FILES) и падал бы
source /opt/ros/humble/setup.bash
source /home/luki/ros2_ws/install/setup.bash 2>/dev/null || true
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=0
cd /home/luki/ros2_ws

echo ">>> [1/5] сборка (rover_drivers, rover_bringup)"
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select rover_drivers rover_bringup \
  > /tmp/rover_build.log 2>&1 && echo "    build OK" || { echo "    BUILD FAIL:"; tail -6 /tmp/rover_build.log; }
source /home/luki/ros2_ws/install/setup.bash

echo ">>> [2/5] глушу старые процессы"
for p in "[r]f2o" "[e]kf" "[s]lam_toolbox" "[r]plidar_scan" "[b]no085" "[e]sp32_cmd" \
         "[r]obot_state" "[r]over_webui" "[d]rivers[.]launch" "[s]lam[.]launch"; do
  pkill -f "$p"
done
sleep 2

LIDAR=/dev/serial/by-id/$(ls /dev/serial/by-id/ 2>/dev/null | grep -i CP2102 | head -1)
ESP=/dev/serial/by-id/$(ls /dev/serial/by-id/ 2>/dev/null | grep -i 1a86 | head -1)
echo ">>> [3/5] порты: LIDAR=$LIDAR  ESP=$ESP"

echo ">>> [4/5] запуск drivers → SLAM → панель"
setsid ros2 launch rover_bringup drivers.launch.py lidar_port:="$LIDAR" esp_port:="$ESP" \
  > ~/drivers.log 2>&1 < /dev/null &
sleep 9
setsid ros2 launch rover_bringup slam.launch.py > ~/slam.log 2>&1 < /dev/null &
setsid ros2 run rover_drivers rover_webui > ~/webui.log 2>&1 < /dev/null &
sleep 12

echo ">>> [5/5] ПРОВЕРКА:"
echo -n "    ESP32: ";  grep -iE "READY" ~/drivers.log | tail -1 | sed 's/.*bridge-3\] //'
echo -n "    камера: "; grep -i "камера открыта" ~/webui.log | tail -1 | sed 's/.*rover_webui\] //'
echo    "    /scan:"; timeout 6 ros2 topic hz /scan 2>/dev/null | head -2 | sed 's/^/      /'
echo    "    /map:";  timeout 6 ros2 topic hz /map  2>/dev/null | head -2 | sed 's/^/      /'
echo "======================================================"
echo "ПАНЕЛЬ:  http://192.168.1.132:8091   (камера + карта + WASD)"
echo "ДАЛЬШЕ:  тест направления лидара —"
echo "  python3 ~/ros2_ws/scripts/scan_bearing.py   # объект ПЕРЕД роботом ~0.5м"
echo "  bearing ~0°=перёд ок · ~180°=задом наперёд → в drivers.launch.py rplidar 'angle_offset_deg':180.0"
