#!/bin/bash
# Madde 12: real network architecture SITL test.
# Each UAV gets its OWN DDS domain (10/11/12) and its OWN uXRCE-DDS agent
# port (8888/8889/8890) - no shared domain, no direct DDS discovery between
# UAVs. The only cross-UAV path is a Zenoh bridge per domain, allowlisted to
# carry ONLY /swarm/* (mission, agent_state, zone, zone_ack) - /fmu/*,
# vision/*, control/*, localization/*, px4/* never leave their own domain.
WS=~/swarm_ws
ZDIR="$WS/tools/zenoh"

echo '>>> Cleaning old processes...'
pkill -9 -x px4 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f MicroXRCEAgent 2>/dev/null
pkill -9 -f zenoh-bridge-ros2dds 2>/dev/null
pkill -9 -f swarm_agent_node 2>/dev/null
pkill -9 -f px4_gateway_node 2>/dev/null
pkill -9 -f localization_node 2>/dev/null
pkill -9 -f vision_node 2>/dev/null
pkill -9 -f image_bridge 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 2

echo '>>> 1/6  uXRCE-DDS agents (one per UAV port - the agent itself takes
no domain flag; it creates its DDS participant in whatever domain the
CONNECTING PX4 client requests, which start_sitl_swarm_multidomain.sh
sets per-instance via ROS_DOMAIN_ID -> PX4s own UXRCE_DDS_DOM_ID param)'
gnome-terminal --title="XRCE-uav0:8888" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
gnome-terminal --title="XRCE-uav1:8889" -- bash -c "MicroXRCEAgent udp4 -p 8889; exec bash"
gnome-terminal --title="XRCE-uav2:8890" -- bash -c "MicroXRCEAgent udp4 -p 8890; exec bash"
sleep 3

echo '>>> 2/6  PX4 SITL + Gazebo (3 drones, per-UAV XRCE port)'
gnome-terminal --title="PX4+Gazebo" -- bash -c "$WS/src/laplacian_swarm/scripts/start_sitl_swarm_multidomain.sh; exec bash"
sleep 18

echo '>>> 3/6  Zenoh bridges (one per domain, /swarm/* allowlist only)'
gnome-terminal --title="Zenoh-d10" -- bash -c "cd $ZDIR && ROS_DOMAIN_ID=10 ./zenoh-bridge-ros2dds peer -d 10 -c bridge_allowlist.json5; exec bash"
gnome-terminal --title="Zenoh-d11" -- bash -c "cd $ZDIR && ROS_DOMAIN_ID=11 ./zenoh-bridge-ros2dds peer -d 11 -c bridge_allowlist.json5; exec bash"
gnome-terminal --title="Zenoh-d12" -- bash -c "cd $ZDIR && ROS_DOMAIN_ID=12 ./zenoh-bridge-ros2dds peer -d 12 -c bridge_allowlist.json5; exec bash"
sleep 3

echo '>>> 4/6  ROS 2 swarm agents (one ros2 launch per UAV, own ROS_DOMAIN_ID)'
gnome-terminal --title="Swarm-uav0(d10)" -- bash -c "source $WS/install/setup.bash && ROS_DOMAIN_ID=10 ros2 launch laplacian_swarm swarm_bringup.launch.py vehicle:=0; exec bash"
gnome-terminal --title="Swarm-uav1(d11)" -- bash -c "source $WS/install/setup.bash && ROS_DOMAIN_ID=11 ros2 launch laplacian_swarm swarm_bringup.launch.py vehicle:=1; exec bash"
gnome-terminal --title="Swarm-uav2(d12)" -- bash -c "source $WS/install/setup.bash && ROS_DOMAIN_ID=12 ros2 launch laplacian_swarm swarm_bringup.launch.py vehicle:=2; exec bash"
sleep 8

echo '>>> 5/6  START trigger (one-shot, from uav0''s domain - relayed to others via Zenoh)'
gnome-terminal --title="START" -- bash -c "source $WS/install/setup.bash && ROS_DOMAIN_ID=10 ros2 run laplacian_swarm mission_trigger --ros-args -p formation:=V -p altitude:=15.0 -p spacing:=5.0; exec bash"

echo '>>> 6/6  QGC (optional monitor, MAVLink - unrelated to ROS domains)'
gnome-terminal --title="QGC" -- bash -c "~/QGroundControl.AppImage; exec bash"

echo 'MULTI-DOMAIN SYSTEM ACTIVE (Madde 12).'
