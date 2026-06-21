#!/bin/bash
# Master launcher - LAPLACIAN swarm (autonomous Gorev 1)
# Spawn positions are set in:
#   src/laplacian_swarm/scripts/start_sitl_swarm.sh  -> PX4_GZ_MODEL_POSE
#   src/laplacian_swarm/launch/swarm_bringup.launch.py -> DRONES spawn_gz  (must match)
WS=~/swarm_ws

echo '>>> Cleaning old processes...'
pkill -9 -x px4 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f MicroXRCEAgent 2>/dev/null
sleep 2

echo '>>> 1/4  uXRCE-DDS agent'
gnome-terminal --title="MicroXRCEAgent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 3

echo '>>> 2/4  PX4 SITL + Gazebo (3 drones)'
gnome-terminal --title="PX4+Gazebo" -- bash -c "$WS/src/laplacian_swarm/scripts/start_sitl_swarm.sh; exec bash"
sleep 18      # 3 PX4 instances + Gazebo need time on a laptop; raise if drones not all up

echo '>>> 3/4  ROS 2 swarm agents + vision (profile:=semi - agents never
self-arm; they wait disarmed until YOU arm+takeoff from QGroundControl, then
SOFTWARE claims Offboard immediately and the mission starts on its own -
no separate manual mode-switch step needed)'
gnome-terminal --title="Swarm" -- bash -c "source $WS/install/setup.bash && ros2 launch laplacian_swarm swarm_bringup.launch.py profile:=semi; exec bash"
sleep 8       # let all 3 agents come up and subscribe before the START latches

echo '>>> 4/4  START trigger (one-shot) - only sets the mission profile/QR
target, does NOT arm anything; the actual mission only begins once you
arm+takeoff all 3 from QGC'
gnome-terminal --title="START" -- bash -c "source $WS/install/setup.bash && ros2 run laplacian_swarm mission_trigger --ros-args -p formation:=V -p altitude:=15.0 -p spacing:=5.0; exec bash"

echo '>>> QGC (use this to Arm + Takeoff each drone - that is all that is needed)'
gnome-terminal --title="QGC" -- bash -c "~/QGroundControl.AppImage; exec bash"

echo 'SYSTEM ACTIVE - waiting for operator arm/takeoff via QGC.'