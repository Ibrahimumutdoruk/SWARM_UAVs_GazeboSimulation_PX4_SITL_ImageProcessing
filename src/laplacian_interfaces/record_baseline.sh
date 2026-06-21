#!/usr/bin/env bash
# Madde 1: nominal SITL akisinin ham kaydi (rosbag). Algoritmaya dokunmaz.
# Kullanim: ./record_baseline.sh   (calistirip mission_trigger sonrasi gorev bitince Ctrl-C)
set -e
source /opt/ros/jazzy/setup.bash
source ~/swarm_ws/install/setup.bash
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=baseline_$STAMP
TOPICS="/swarm/agent_state /swarm/mission \
  /uav0/fmu/out/vehicle_local_position_v1 /uav1/fmu/out/vehicle_local_position_v1 /uav2/fmu/out/vehicle_local_position_v1 \
  /uav0/fmu/out/vehicle_status_v4 /uav1/fmu/out/vehicle_status_v4 /uav2/fmu/out/vehicle_status_v4 \
  /uav0/vision/qr /uav1/vision/qr /uav2/vision/qr \
  /uav0/vision/color /uav1/vision/color /uav2/vision/color"
echo "recording -> $OUT  (Ctrl-C when mission done)"
ros2 bag record -o "$OUT" $TOPICS
