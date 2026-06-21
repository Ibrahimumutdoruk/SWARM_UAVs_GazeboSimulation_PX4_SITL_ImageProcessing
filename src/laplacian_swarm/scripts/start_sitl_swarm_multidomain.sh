#!/usr/bin/env bash
# Madde 12: same 3-drone SITL spawn as start_sitl_swarm.sh, but each PX4
# instance's uXRCE-DDS client connects to its OWN MicroXRCEAgent port
# (8888/8889/8890) instead of sharing one, AND requests its own DDS domain
# (10/11/12) - PX4's own rcS auto-sets UXRCE_DDS_DOM_ID from $ROS_DOMAIN_ID
# at boot, so the agent ends up creating its participant in that same
# domain (driven by the client's request, no agent-side override needed).
# This is the real per-UAV-domain network architecture, not the
# single-shared-domain SITL convenience setup.

pkill -9 -x px4 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
sleep 1   # clean slate

export PX4_DIR=$HOME/PX4-Autopilot
export AIRFRAME=4001
export MODEL=gz_x500
export WORLD=default

export PX4_HOME_LAT=38.325954
export PX4_HOME_LON=33.992143
export PX4_HOME_ALT=980.0

# ======================================================
cd "$PX4_DIR"

run() {
  gnome-terminal -- bash -c \
  "export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia; \
   PX4_SYS_AUTOSTART=$AIRFRAME PX4_SIM_MODEL=$MODEL PX4_GZ_WORLD=$WORLD \
   PX4_GZ_MODEL_POSE='$2' PX4_UXRCE_DDS_NS=$3 PX4_UXRCE_DDS_PORT=$5 ROS_DOMAIN_ID=$6 $4 ./build/px4_sitl_default/bin/px4 -i $1; exec bash";
}

run 0 "106.98,35.0,0.1,0,0,-1.57" uav0 "" 8888 10
sleep 8
run 1 "105.0,34.95,0.1,0,0,-1.57" uav1 "PX4_GZ_STANDALONE=1" 8889 11
run 2 "103.41,34.9,0.1,0,0,-1.57" uav2 "PX4_GZ_STANDALONE=1" 8890 12
