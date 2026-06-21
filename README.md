<h1 align="center">🚁 SWARM UAVs — Gazebo PX4 SITL Simulation & Image Processing</h1>

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

## ✨ Highlights

- 🧠 **Fully decentralized** — every drone runs its own ROS 2 agent and FSM; there is **no central commander**. The swarm coordinates through broadcast messages.
- 🎯 **Formation flight** — V / line formations with configurable altitude and spacing.
- 👁️ **Image processing** — OpenCV pipeline on each drone's downward camera:
  - **QR mission decoding** — the mission is read live from a QR code in the world.
  - **Color zone detection & fusion** — drones detect and agree on colored zones.
- 🔀 **Split → search → rejoin** — the formation can break apart to search and then reform.
- 🛡️ **Collision avoidance & autonomous return**.
- 🔁 **One codebase for sim & real** — switch behavior with a single profile (`sitl` / `semi` / `real`).

---

## 📸 Screenshots & Gallery

> Simulation captures (Gazebo + image-processing output). Drop your images in [`docs/images/`](docs/images/) and they'll render here.

<p align="center">
  <!-- Replace these with your own screenshots -->
  <img src="docs/images/gazebo_swarm.png"   alt="3 drones in Gazebo SITL"        width="45%">
  <img src="docs/images/formation.png"       alt="V formation flight"             width="45%">
</p>
<p align="center">
  <img src="docs/images/qr_detection.png"    alt="QR mission detection (OpenCV)"  width="45%">
  <img src="docs/images/color_zones.png"     alt="Color zone detection"           width="45%">
</p>

| Image | What it shows |
|-------|---------------|
| `gazebo_swarm.png` | The 3 PX4 drones spawned in the Gazebo world |
| `formation.png`    | Drones holding a V formation |
| `qr_detection.png` | OpenCV decoding the mission QR code from the down camera |
| `color_zones.png`  | Detected/fused color zones |

*(Filenames above are placeholders — add your own and update the table.)*

---

## 🧰 Tech Stack / Versions

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

## 📦 Repository Layout

```
.
├── README.md
├── launch_swarm.sh            # one-command launcher (XRCE + PX4/Gazebo + agents + QGC)
├── start_multidomain.sh       # per-UAV DDS-domain launcher (multi-machine topology)
├── record_baseline.sh         # capture nominal behavior for regression
├── regression_logger.py
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

### Nodes (`laplacian_swarm`)
| Node | Role |
|------|------|
| `swarm_agent_node` | Decentralized per-drone brain / FSM (formation, separation, QR mission, color search, split/rejoin, return) |
| `px4_gateway_node` | The **only** node that touches `/fmu/*` (arm/offboard/setpoints) |
| `localization_node` | Converts PX4 GNSS → shared `FIELD_ENU` frame |
| `vision_node` | OpenCV QR + color detection from the downward camera |
| `mission_trigger` | One-shot START: latches formation / altitude / QR target |

---

## 🔧 Dependencies

```bash
# ROS 2 Jazzy core
sudo apt install ros-jazzy-desktop

# ROS ↔ Gazebo bridges + image transport + vision
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-image ros-jazzy-cv-bridge

# message tooling + build tools
sudo apt install ros-jazzy-rosidl-default-generators ros-jazzy-std-msgs ros-jazzy-sensor-msgs \
                 python3-colcon-common-extensions python3-rosdep
```

**Python libs:** `opencv-python` (4.13.0), `numpy` (1.26.4), `pyyaml`, `setuptools`.

**External tools (installed separately):** PX4-Autopilot (SITL + Gazebo), Micro XRCE-DDS Agent, QGroundControl.

---

## 🏗️ Build

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

---

## ▶️ Run the Simulation

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

## ⚙️ Configuration & Profiles

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

> ⚠️ The `spawn_gz` poses in `vehicles.yaml` **must match** `PX4_GZ_MODEL_POSE` in `scripts/start_sitl_swarm.sh`.

---

## 🩺 Troubleshooting

- **Not all 3 drones spawn** → increase the `sleep 18` in `launch_swarm.sh` (3 PX4 + Gazebo need time on a laptop).
- **Agents idle after launch** → in `semi` profile you must Arm + Takeoff each drone from QGroundControl.
- **No PX4 topics** → confirm `MicroXRCEAgent udp4 -p 8888` is running and `px4_msgs` is built.
- **Camera topic missing** → install `ros-jazzy-ros-gz-image`; the bridge remaps the Gazebo camera to `/uavX/down_cam/image`.

---

## 📄 License

MIT.

## 👤 Author

**Ibrahim Umut Doruk** · [@Ibrahimumutdoruk](https://github.com/Ibrahimumutdoruk)
