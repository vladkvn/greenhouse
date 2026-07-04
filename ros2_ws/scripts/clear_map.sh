#!/usr/bin/env bash
# Очистка карты slam_toolbox с ЧИСТЫМ рестартом локализации.
#
# Почему так, а не «просто перезапустить slam»: slam.launch поднимает rf2o+EKF+slam
# ОДНОВРЕМЕННО, а rf2o без энкодеров в первые ~2с (пока «Configuring node») выдаёт «дикую»
# одометрию. Если slam уже слушает — эти сканы впекаются в свежую карту под случайными позами
# и она мгновенно «размазывается» на десятки метров (map->odom уезжает на 30+ м). Поэтому
# чистим ПОСЛЕДОВАТЕЛЬНО: сперва локализация, ждём сходимости rf2o, ТОЛЬКО потом slam_toolbox —
# он стартует уже на стабильной одометрии и строит чистую карту с текущей позы робота (0,0).
#
# drivers / nav2 / webui НЕ трогаем. Активная цель Nav2, если была, отменится (map->odom мигнёт).
# Запуск:  bash ~/ros2_ws/scripts/clear_map.sh
# БЕЗ set -u (ROS setup.bash использует unbound-переменные). pkill-паттерны bracket-безопасны;
# в файле-скрипте self-match не грозит.
source /opt/ros/humble/setup.bash
source /home/luki/ros2_ws/install/setup.bash 2>/dev/null || true
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=0
YAML=/home/luki/ros2_ws/src/rover_bringup/config/slam_toolbox.yaml
cd /home/luki/ros2_ws

echo ">>> глушу slam + localization: rf2o / EKF / slam"
pkill -f "[s]lam[.]launch"
pkill -f "[l]ocalization[.]launch"
pkill -f "[a]sync_slam_toolbox"
pkill -f "[r]f2o_laser"
pkill -f "[e]kf_node"
sleep 3

echo ">>> старт локализации rf2o+EKF, жду сходимости rf2o ~9с"
setsid ros2 launch rover_bringup localization.launch.py > ~/loc.log 2>&1 < /dev/null &
sleep 9

echo ">>> старт slam_toolbox на стабильной одометрии - чистая карта"
setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$YAML" -p use_sim_time:=false > ~/slam_node.log 2>&1 < /dev/null &
sleep 7

echo ">>> размер новой карты - должен быть МАЛЕНЬКИМ, порядка комнаты:"
timeout 6 ros2 topic echo /map --field info --once 2>/dev/null | grep -E "width|height"
echo "======================================================"
echo "готово - карта чистая. Кати МЕДЛЕННО; повороты и проезд дверей - самые опасные для rf2o."
