#!/usr/bin/env bash
# ПОЛНЫЙ чистый подъём стека ровера (Jetson): drivers + локализация/slam + nav2 + панель +
# восприятие(person) + follow. Чистит stale DDS SHM (копится от множества рестартов и роняет
# ros2-CLI: "RTPS_TRANSPORT_SHM ... open_and_lock_file failed"). Даёт СВЕЖУЮ карту.
#
# Запуск: bash ~/ros2_ws/scripts/rover_up_full.sh
# БЕЗ set -u (ROS setup.bash использует unbound). pkill-паттерны bracket-безопасны.
source /opt/ros/humble/setup.bash
source /home/luki/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
VENV=/home/luki/greenhouse/.venv/bin/python
MODEL=/home/luki/greenhouse/models/yolo11n.engine
YAML=/home/luki/ros2_ws/src/rover_bringup/config/slam_toolbox.yaml
cd /home/luki/ros2_ws

echo ">>> [1/6] глушу всё"
for p in "[r]f2o" "[e]kf_node" "[a]sync_slam" "[c]ontroller_server" "[p]lanner_server" \
         "[b]t_navigator" "[b]ehavior_server" "[w]aypoint" "[l]ifecycle_manager" \
         "[p]erson_tracker" "[f]ollow_behavior" "[r]over_webui" "[c]amera_node" \
         "[v]esc_diff_drive" "[r]plidar" "[r]obot_state" "[b]no085" "[s]lam[.]launch" \
         "[l]ocalization[.]launch" "[d]rivers[.]launch" "[n]av2[.]launch"; do pkill -f "$p"; done
sleep 3
echo ">>> чищу stale DDS SHM"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

LIDAR=/dev/serial/by-id/$(ls /dev/serial/by-id/ 2>/dev/null | grep -i CP2102 | head -1)
ESP=/dev/serial/by-id/$(ls /dev/serial/by-id/ 2>/dev/null | grep -i 1a86 | head -1)
echo ">>> [2/6] drivers: лидар, мост(255/50), IMU, камера  ($LIDAR / $ESP)"
setsid ros2 launch rover_bringup drivers.launch.py lidar_port:="$LIDAR" esp_port:="$ESP" \
  > ~/drivers.log 2>&1 < /dev/null &
sleep 10

echo ">>> [3/6] локализация rf2o+EKF, жду сходимости rf2o ~9с"
setsid ros2 launch rover_bringup localization.launch.py > ~/loc.log 2>&1 < /dev/null &
sleep 9
echo ">>> slam_toolbox (чистая карта на стабильной одометрии)"
setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$YAML" -p use_sim_time:=false > ~/slam_node.log 2>&1 < /dev/null &
sleep 6

echo ">>> [4/6] Nav2"
setsid ros2 launch rover_bringup nav2.launch.py > ~/nav2.log 2>&1 < /dev/null &
echo ">>> [5/6] панель"
setsid ros2 run rover_drivers rover_webui > ~/webui.log 2>&1 < /dev/null &
# Восприятие (person+follow, YOLO) — тяжёлое, OOM-риск: только по аргументу 'follow'
if [ "${1:-}" = "follow" ]; then
  echo ">>> [6/6] восприятие: person(venv) + follow"
  setsid bash -c "unset PYTHONNOUSERSITE; exec $VENV -m rover_drivers.person_tracker \
    --ros-args -p model_path:=$MODEL -p conf:=0.4 -p rate_hz:=12.0" > ~/person.log 2>&1 < /dev/null &
  setsid ros2 run rover_drivers follow_behavior > ~/follow.log 2>&1 < /dev/null &
else
  echo ">>> [6/6] восприятие ПРОПУЩЕНО (запусти 'bash rover_up_full.sh follow' чтобы включить)"
fi
sleep 16

echo "======================================================"
echo -n "nav2 active: "; grep -c "Managed nodes are active" ~/nav2.log
echo -n "footprint:   "; timeout 6 ros2 param get /global_costmap/global_costmap robot_radius 2>/dev/null | grep -i double
echo -n "person YOLO: "; grep -c "YOLO загружен" ~/person.log
echo -n "follow:      "; timeout 5 ros2 topic echo /follow/state --once 2>/dev/null | grep data
echo "ПАНЕЛЬ: http://192.168.1.132:8091   ·   карта свежая, кати медленно"
