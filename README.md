<h1 align="center"> SWARM UAVs — Gazebo PX4 SITL Simulation & Image Processing</h1>

<p align="center">
  <b>A decentralized 3× drone autonomous swarm</b> flying in <b>Gazebo SITL</b> on <b>PX4 Offboard</b>,
  coordinating peer-to-peer over <b>ROS 2</b>, with onboard <b>OpenCV image processing</b>
  (QR mission reading + color zone detection).
</p>

<p align="center">
  <img alt="ROS 2 Jazzy"   src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white">
  <img alt="Gazebo"        src="https://img.shields.io/badge/Gazebo_Sim-8.11_(Harmonic)-FF6B00">
  <img alt="PX4"           src="https://img.shields.io/badge/PX4-SITL-026FB4">
  <img alt="Python"        src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white">
  <img alt="OpenCV"        src="https://img.shields.io/badge/OpenCV-4.13-5C3EE8?logo=opencv&logoColor=white">
  <img alt="License"       src="https://img.shields.io/badge/License-MIT-green">
</p>

---

##  Highlights

-  **Fully decentralized** — every drone runs its own ROS 2 agent and FSM; there is **no central commander**. The swarm coordinates through broadcast messages.
-  **Formation flight** — V / line formations with configurable altitude and spacing.
-  **Image processing** — OpenCV pipeline on each drone's downward camera:
  - **QR mission decoding** — the mission is read live from a QR code in the world.
  - **Color zone detection & fusion** — drones detect and agree on colored zones.
-  **Split → search → rejoin** — the formation can break apart to search and then reform.
-  **Collision avoidance & autonomous return**.
-  **One codebase for sim & real** — switch behavior with a single profile (`sitl` / `semi` / `real`).

---

## Gallery


<p align="center">
  <img src="docs/images/Formations.png"    alt="Swarm formation flight"             width="45%">
  <img src="docs/images/Blue_Red_Area.png" alt="Blue/red color zone detection"      width="45%">
</p>
<p align="center">
  <img src="docs/images/QGC.png"           alt="QGroundControl arm and takeoff"     width="60%">
</p>

---

## Tech Stack / Versions

Developed & tested on this exact stack — matching it is recommended.

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 24.04 LTS (x86_64) |
| **ROS 2** | Jazzy Jalisco |
| **Gazebo** | Gazebo Sim 8.11.0 (Harmonic) — the new `gz sim` |
| **PX4-Autopilot** | `main` (SITL, airframe `4001`, model `gz_x500`) |
| **Micro XRCE-DDS Agent** | latest (`MicroXRCEAgent`, UDP `8888`) |
| **Python** | 3.12 |
| **OpenCV (cv2)** | 4.13.0 |
| **NumPy** | 1.26.4 |
| **QGroundControl** | AppImage (operator arm + takeoff) |
| **Build** | colcon (`ament_cmake` + `ament_python` + `rosidl`) |

---

##  Repository Layout

```
.
├── README.md
├── launch_swarm.sh            # one-command launcher (XRCE + PX4/Gazebo + agents + QGC)
├── start_multidomain.sh       # per-UAV DDS-domain launcher (multi-machine topology)
├── record_baseline.sh         # capture nominal behavior for regression
├── regression_logger.py
├── tools/zenoh/               # Zenoh bridge allowlist (/swarm/* only); drop the bridge binary here
├── simulation/gz/             # Gazebo world (default.sdf) + x500 down-camera model + field models
├── docs/images/               # ← put your simulation screenshots here
└── src/
    ├── laplacian_interfaces/  # custom ROS 2 messages (ament_cmake / rosidl)
    │   └── msg/                # AgentState, Px4Status, QrDetection, ColorDetection, ...
    ├── laplacian_swarm/       # the swarm app (ament_python)
    │   ├── laplacian_swarm/    # nodes (see below)
    │   ├── config/             # vehicles.yaml, field.yaml, profile_*.yaml
    │   ├── launch/             # swarm_bringup.launch.py
    │   └── scripts/            # start_sitl_swarm*.sh
    └── px4_msgs/              # PX4 uORB ↔ ROS 2 messages (git submodule)
```

### Packages
| Package | Build type | Purpose |
|---------|-----------|---------|
| `laplacian_interfaces` | `ament_cmake` + `rosidl` | Custom swarm broadcast + telemetry messages |
| `laplacian_swarm` | `ament_python` | All swarm nodes, launch files, and configs |
| `px4_msgs` | `ament_cmake` | PX4 uORB ↔ ROS 2 messages (git submodule) |

### Nodes (`laplacian_swarm`)
| Node | Role |
|------|------|
| `swarm_agent_node` | Decentralized per-drone brain / FSM (formation, separation, QR mission, color search, split/rejoin, return) |
| `px4_gateway_node` | The **only** node that touches `/fmu/*` (arm/offboard/setpoints) |
| `localization_node` | Converts PX4 GNSS → shared `FIELD_ENU` frame |
| `vision_node` | OpenCV QR + color detection from the downward camera |
| `mission_trigger` | One-shot START: latches formation / altitude / QR target |

---

## Node Architecture

```
                 ┌──────────────────────────────────────────────┐
   PX4 (SITL/HW) │  /fmu/*  (uORB over uXRCE-DDS, UDP 8888)      │
                 └──────────────┬───────────────────────────────┘
                                │  (ONLY this node touches /fmu/*)
                       ┌────────▼─────────┐
                       │ px4_gateway_node │  arm/offboard/setpoints + Px4Status
                       └────────┬─────────┘
   GNSS ───────────────► localization_node ──► LocalizationState (FIELD_ENU)
   down camera ────────► vision_node ────────► QrDetection / ColorDetection
                                │
                       ┌────────▼─────────┐    broadcast AgentState / Zone* / MissionCommand
                       │ swarm_agent_node │◄──────────► (other 2 agents, decentralized)
                       └──────────────────┘
                                ▲
                       mission_trigger (one-shot START)
```

- **`swarm_agent_node`** — the decentralized FSM: formation math, separation/collision, QR mission logic, color search, split/rejoin, return. Runs identically per drone.
- **`px4_gateway_node`** — sole owner of the PX4 interface; isolates `/fmu/*` so the rest of the code is hardware-agnostic.
- **`localization_node`** — converts PX4 GNSS to the shared `FIELD_ENU` frame all agents agree on.
- **`vision_node`** — OpenCV-based QR decoding + color zone detection from the downward camera.
- **`mission_trigger`** — latches the mission parameters once; does **not** arm anything.

---

## Custom Messages

Defined in `src/laplacian_interfaces/msg/`:

| Message | Role |
|---------|------|
| `AgentState.msg` | Per-agent broadcast: pose, FSM state, health |
| `Px4Status.msg` | PX4 health / arming / land-detector status |
| `LocalizationState.msg` | Position in shared `FIELD_ENU` frame |
| `ControlSetpoint.msg` | Desired setpoint to the gateway |
| `ControlCommand.msg` | Arm / offboard / mode command |
| `CommandResult.msg` | Result / ack of a control command |
| `MissionCommand.msg` | Mission propose / commit |
| `QrDetection.msg` | Decoded QR mission payload |
| `ColorDetection.msg` | Detected color observation |
| `ZoneObservation.msg` | Color-zone observation for fusion |
| `ZoneAck.msg` | Zone fusion acknowledgement |

---

##  Dependencies

```bash
# ROS 2 Jazzy core
sudo apt install ros-jazzy-desktop

# ROS ↔ Gazebo bridges + image transport + vision
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-image ros-jazzy-cv-bridge

# message tooling + build tools
sudo apt install ros-jazzy-rosidl-default-generators ros-jazzy-std-msgs ros-jazzy-sensor-msgs \
                 python3-colcon-common-extensions python3-rosdep
```

**ROS dependencies (from `package.xml`):** `rclpy`, `std_msgs`, `sensor_msgs`, `cv_bridge`, `ros_gz_image`, `px4_msgs`, `laplacian_interfaces`.

**Python libs:** `opencv-python` (4.13.0), `numpy` (1.26.4), `pyyaml`, `setuptools`.

**External tools (installed separately):** PX4-Autopilot (SITL + Gazebo), Micro XRCE-DDS Agent, QGroundControl.

---

##  Build

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Clone (--recursive pulls in the px4_msgs submodule)
git clone --recursive https://github.com/Ibrahimumutdoruk/SWARM_UAVs_GazeboSimulation_PX4_SITL_ImageProcessing.git swarm_ws
cd swarm_ws
#   if you forgot --recursive:  git submodule update --init --recursive

# 3. Resolve deps + build
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> Building selectively? Build `laplacian_interfaces` before `laplacian_swarm`:
> `colcon build --packages-select laplacian_interfaces && colcon build --packages-select laplacian_swarm`

---

##  Run the Simulation

### One command
```bash
./launch_swarm.sh
```
This opens terminals and starts, in order:
1. **Micro XRCE-DDS Agent** — `MicroXRCEAgent udp4 -p 8888`
2. **PX4 SITL + Gazebo** — 3 drones (`uav0`, `uav1`, `uav2`)
3. **ROS 2 swarm agents + vision** — `profile:=semi`
4. **Mission trigger** — formation `V`, altitude `15 m`, spacing `5 m`
5. **QGroundControl** — Arm + Takeoff each drone

> In the `semi` profile the agents **never self-arm**: they wait disarmed until you **Arm + Takeoff** from QGroundControl, then the software claims Offboard and the mission runs autonomously.

### Manual (equivalent)
```bash
# T1
MicroXRCEAgent udp4 -p 8888
# T2
./src/laplacian_swarm/scripts/start_sitl_swarm.sh
# T3
source install/setup.bash
ros2 launch laplacian_swarm swarm_bringup.launch.py profile:=sitl   # or profile:=semi
# T4 — start the mission
ros2 run laplacian_swarm mission_trigger --ros-args -p formation:=V -p altitude:=15.0 -p spacing:=5.0
```

---

##  Configuration & Profiles

All config lives in `src/laplacian_swarm/config/`.

| File | Purpose |
|------|---------|
| `vehicles.yaml` | Per-UAV manifest: id, system_id, namespace, Gazebo spawn pose, slot, DDS domain, XRCE port |
| `field.yaml` | Field origin (lat/lon/alt) for the shared `FIELD_ENU` frame |
| `profile_sitl.yaml` | Full SITL — agents self-arm, simulated GNSS |
| `profile_semi.yaml` | SITL but operator arms via QGC (no self-arm) |
| `profile_real.yaml` | Real flight — arm/offboard from RC only, physical camera, F9P GNSS |

```bash
ros2 launch laplacian_swarm swarm_bringup.launch.py profile:=semi
```

> The `spawn_gz` poses in `vehicles.yaml` **must match** `PX4_GZ_MODEL_POSE` in `scripts/start_sitl_swarm.sh`.

---

## Simulation World & Camera

The Gazebo assets live in [`simulation/gz/`](simulation/gz/):

```
simulation/gz/
├── worlds/default.sdf      # the competition world
└── models/
    ├── x500/               # x500 drone + downward QR/color camera (model.sdf)
    ├── mavi_alan/          # blue area
    ├── kirmizi_alan/       # red area
    └── qr_11 .. qr_66/     # QR mission markers
```

### The world (`worlds/default.sdf`)

`default.sdf` is the custom field used for the mission. It includes the colored areas the drones detect and fuse, plus the QR markers that carry the mission:

```xml
<include><name>mavi_alan</name>    <uri>model://mavi_alan</uri>    <pose>105 0  0.08 0 0 0</pose></include>
<include><name>kirmizi_alan</name> <uri>model://kirmizi_alan</uri> <pose>117 5  0.08 0 0 0</pose></include>
<include><name>qr_11</name> <uri>model://qr_11</uri> <pose>98  0  0.1 0 0 0</pose></include>
<include><name>qr_22</name> <uri>model://qr_22</uri> <pose>112 0  0.1 0 0 0</pose></include>
<!-- qr_33 .. qr_66 -->
```

- `mavi_alan` = blue area, `kirmizi_alan` = red area — the targets for the color zone detection / fusion.
- `qr_11 … qr_66` = the QR markers the `vision_node` decodes for the mission.

### The downward camera (`models/x500/model.sdf`)

Each drone carries a downward-facing camera (`fpv_camera`) on a dedicated `camera_link`, used by `vision_node` for QR decoding and color detection. The sensor settings:

```xml
<!-- downward QR/color camera -->
<link name="camera_link">
  <pose relative_to="base_link">0.2 0 -0.05 0 0 0</pose>   <!-- mounted under the body -->
  <sensor name="fpv_camera" type="camera">
    <pose>0 0 0 0 1.5708 0</pose>                           <!-- pitched 90° to look straight down -->
    <camera>
      <horizontal_fov>1.2217305</horizontal_fov>            <!-- ~70° -->
      <image><width>1920</width><height>1080</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>1000</far></clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>90</update_rate>
    <visualize>true</visualize>
  </sensor>
</link>
```

| Setting | Value | Notes |
|---------|-------|-------|
| Resolution | 1920 × 1080 (RGB) | `R8G8B8` |
| Horizontal FOV | 1.2217 rad (~70°) | |
| Orientation | pitched 1.5708 rad (90°) | looks straight down |
| Mount | `0.2 0 -0.05` rel. to `base_link` | under the airframe |
| Update rate | 90 Hz | |
| Clip | 0.1 m – 1000 m | |

The Gazebo image topic is bridged into ROS 2 (via `ros_gz_image`) and remapped to `/uavX/down_cam/image`, which `vision_node` subscribes to.

### Installing the assets

Gazebo finds models/worlds via path env vars. Point them at this folder (or copy into your PX4 sim tree):

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/simulation/gz/models:$PWD/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
```

> The drone model in `start_sitl_swarm.sh` is `gz_x500`; the `model.sdf` here is the matching x500 with the downward camera link added.

---

## Networking — DDS vs Zenoh Bridge

The swarm can run in two network topologies. Both use the exact same nodes and code; only the transport changes.

| Mode | When | Transport |
|------|------|-----------|
| **Single shared DDS domain** | Default SITL on one machine (`launch_swarm.sh`) | All UAVs share one `ROS_DOMAIN_ID`; direct DDS discovery |
| **Per-UAV domain + Zenoh bridge** | Multi-machine / real-hardware topology (`start_multidomain.sh`) | Each UAV gets its **own** `ROS_DOMAIN_ID` (10/11/12) and its own uXRCE-DDS port (8888/8889/8890); no direct DDS discovery between UAVs |

### How the Zenoh bridge works

In the multi-domain mode there is **no shared DDS bus**. The only path between drones is one `zenoh-bridge-ros2dds` instance per domain. The bridge is allowlisted to carry **only** the `/swarm/*` topics (mission, agent_state, zone, zone_ack) across drones. Everything private to a drone — `/fmu/*`, `vision/*`, `control/*`, `localization/*`, `px4/*` — never leaves its own domain.

This mirrors the real hardware setup, where each drone is a separate computer on a Wi-Fi mesh (`bat0`) and the drones discover each other over Zenoh instead of plain DDS multicast. On the `real` profile this is selected by `network_mode: zenoh_bat0` in `profile_real.yaml`.

The allowlist lives in [`tools/zenoh/bridge_allowlist.json5`](tools/zenoh/bridge_allowlist.json5):

```json5
{
  plugins: {
    ros2dds: {
      allow: {
        publishers:  ["/swarm/.*"],
        subscribers: ["/swarm/.*"],
      },
    },
  },
}
```

### Running the Zenoh / multi-domain mode

1. Get the bridge binary — download `zenoh-bridge-ros2dds` (from the [eclipse-zenoh/zenoh-plugin-ros2dds releases](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases)) and place it in `tools/zenoh/`.
2. Launch everything:
   ```bash
   ./start_multidomain.sh
   ```
   This starts, per UAV: a uXRCE-DDS agent on its own port, PX4+Gazebo, a Zenoh bridge bound to its domain with the `/swarm/*` allowlist, and the swarm nodes. One bridge per domain peers with the others to relay only the swarm topics.

Manual single-bridge example for one domain:
```bash
cd tools/zenoh
ROS_DOMAIN_ID=10 ./zenoh-bridge-ros2dds peer -d 10 -c bridge_allowlist.json5
```

---

##  Troubleshooting

- **Not all 3 drones spawn** → increase the `sleep 18` in `launch_swarm.sh` (3 PX4 + Gazebo need time on a laptop).
- **Agents idle after launch** → in `semi` profile you must Arm + Takeoff each drone from QGroundControl.
- **No PX4 topics** → confirm `MicroXRCEAgent udp4 -p 8888` is running and `px4_msgs` is built.
- **Camera topic missing** → install `ros-jazzy-ros-gz-image`; the bridge remaps the Gazebo camera to `/uavX/down_cam/image`.
- **Drones don't see each other in multi-domain mode** → make sure `zenoh-bridge-ros2dds` is in `tools/zenoh/` and a bridge is running for each domain; only `/swarm/*` crosses domains by design.

---
