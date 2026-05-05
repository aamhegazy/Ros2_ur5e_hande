# Mantis — Raspberry Pi 4 Unity↔ROS 2 Bridge

This branch (`raspberry-pi`) turns a **Raspberry Pi 4** into a lightweight ROS 2 bridge for the Mantis XR teleoperation project. The Pi exposes:

1. A **Unity ↔ ROS 2 TCP bridge** — Unity (Quest/Windows) connects via `ROS-TCP-Endpoint` and exchanges action goals/results for `PlanToPose` and `ExecutePlan`.
2. A **RealSense D435i ROS 2 publisher** — color and depth streams published as `sensor_msgs/Image` topics, viewable in any browser via `web_video_server`.

The Pi runs a **fake action server** that mocks the MoveIt planner. This makes it possible to develop and test the full Unity XR → ROS 2 pipeline **without the Ubuntu PC, without MoveIt, and without the physical UR5e**.

> For real hardware control (UR5e + Hand-E + MoveIt), use the `main` branch on the Ubuntu PC.

---

## 1. What's on the Pi

| Package | Role |
|---|---|
| `ur5e_moveit_actions_pi` | Pi-only action message definitions (`PlanToPose`, `ExecutePlan`) + launch file + Pi nodes. No MoveIt dependency. |
| `ROS-TCP-Endpoint` (patched) | Unity TCP bridge on port 10000. |
| `realsense2_camera` | (Optional) Stock RealSense ROS 2 wrapper — known unstable on Pi 4 + D435i. |

### Pi-specific scripts (in `ur5e_moveit_actions/src/`, prefixed `rpi_`)

| Script | Purpose |
|---|---|
| `rpi_fake_action_server.py` | Mocks MoveIt — accepts `PlanToPose` / `ExecutePlan` goals, returns dummy success. |
| `rpi_unity_action_bridge.py` | Bridges Unity topic publishes (`plan_to_pose/goal`, `execute_plan/goal`) to ROS 2 action calls and forwards results back. |
| `rpi_realsense_publisher.py` | Custom RealSense publisher using `pyrealsense2` directly (workaround for Pi UVC issues). |

---

## 2. Network layout

| Device | IP | Role |
|---|---|---|
| Mobile hotspot | `172.20.10.1` | Gateway |
| Raspberry Pi 4 | `172.20.10.2` | Unity TCP bridge + RealSense publisher |
| Windows PC (Unity Editor / Quest) | `172.20.10.6` | XR frontend |

SSH into the Pi:
```bash
ssh hgz-rpi4@172.20.10.2
```

---

## 3. One-command launch (recommended)

The launch file starts the fake action server, the Unity action bridge, and the TCP endpoint together:

```bash
ros2 launch ur5e_moveit_actions_pi rpi_bridge.launch.py
```

With explicit IP override:
```bash
ros2 launch ur5e_moveit_actions_pi rpi_bridge.launch.py ros_ip:=172.20.10.2
```

What gets started:

| Node | Package | Purpose |
|---|---|---|
| `fake_action_server` | `ur5e_moveit_actions_pi` | Mock plan/execute server |
| `unity_action_bridge` | `ur5e_moveit_actions_pi` | Topic ↔ action relay |
| `ros_tcp_endpoint` | `ros_tcp_endpoint` | Unity TCP server on `:10000` |

### Verify topics

```bash
ros2 topic list
# Expected:
# /execute_plan/goal
# /execute_plan/result
# /plan_to_pose/goal
# /plan_to_pose/result
```

---

## 4. Unity-side setup

In Unity:

| Setting | Value |
|---|---|
| ROS IP | `172.20.10.2` |
| Port | `10000` |
| Protocol | ROS 2 |

Generated message classes use the namespace **`ur5e_moveit_actions/action/...`** (without the `_pi` suffix). The Pi resolves this transparently — see §6.

---

## 5. Action-message format

### PlanToPose
geometry_msgs/PoseStamped target_pose
bool success
string message
string plan_id
geometry_msgs/Point[] tcp_waypoints
float32 progress

### ExecutePlan
string plan_id
bool success
string message
float32 progress

---

## 6. ROS-TCP-Endpoint patch

The upstream `default_server_endpoint` only resolves 2-part message names like `package/MessageName`. Unity sends 3-part action names like `ur5e_moveit_actions/action/PlanToPose_Goal`.

A patch was applied to:
~/ur5e_hande_ws/install/ros_tcp_endpoint/lib/python3.10/site-packages/ros_tcp_endpoint/server.py

The patched `resolve_message_name()` handles 3-part action names by splitting the inner type into `<ActionClass>_<Goal|Result|Feedback>` and resolving the inner class via `getattr(action_cls, inner_class_name)`.

A symlink also exposes `ur5e_moveit_actions_pi` under the name Unity uses:

```bash
cd ~/ur5e_hande_ws/install/ur5e_moveit_actions_pi/local/lib/python3.10/dist-packages/
ln -s ur5e_moveit_actions_pi ur5e_moveit_actions
```

If the patch is lost (e.g. after rebuilding `ros_tcp_endpoint`), re-apply it from `docs/patches/server.py.patch` (if maintained) or re-edit `resolve_message_name()` manually.

---

## 7. RealSense D435i — `rpi_realsense_publisher.py`

The stock `realsense-ros` wrapper is unstable on the Pi 4 + D435i combination — `xioctl(UVCIOC_CTRL_QUERY)` errors on UVC extension unit control 1 prevent reliable streaming, even with `enable_sync:=false` and reduced resolutions.

**Workaround:** `rpi_realsense_publisher.py` uses `pyrealsense2` directly and publishes to ROS 2 topics manually.

### Topics

| Topic | Type |
|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` |

### Run

```bash
source /opt/ros/humble/setup.bash
python3 ~/ur5e_hande_ws/src/Ros2_ur5e_hande/rpi_realsense_publisher.py
```

### View in browser (no display needed)

```bash
# In a second terminal
ros2 run web_video_server web_video_server
```

Open in any browser:
http://172.20.10.2:8080/stream?topic=/camera/color/image_raw

### Requirements

- `pyrealsense2` system-wide: `sudo pip3 install pyrealsense2 "numpy<2"`
- `numpy<2` is required — ROS 2 `cv_bridge` is incompatible with numpy 2.x.
- Camera must be connected to a **USB 3.0 port** (blue) on the Pi. USB 2.0 causes bandwidth issues and timeouts.
- librealsense built from source with `-DFORCE_RSUSB_BACKEND=ON` (see §9).

---

## 8. End-to-end test from Unity

1. Pi: `ros2 launch ur5e_moveit_actions_pi rpi_bridge.launch.py`
2. Unity (Editor or Quest build): set ROS IP `172.20.10.2:10000`, enter Play mode.
3. In the XR scene, point the right controller's near-far ray at a target location, press **grip** to spawn a plan target.
4. Point at the spawned menu → press **Plan**. Expect on the Pi terminal:
[unity_action_bridge] Relaying plan goal to /plan_to_pose action
[fake_action_server] Received plan goal: position=(x, y, z)
[unity_action_bridge] plan done: success=True
5. The **Execute** button on the menu turns blue. Press it — expect:
[unity_action_bridge] Relaying execute goal to /execute_plan action
[fake_action_server] Received execute goal: plan_id=fake_plan_001
[unity_action_bridge] execute done: success=True

If any step fails, see §10 Troubleshooting.

---

## 9. First-time Pi setup (reference)

The Pi is already configured. These steps are documented in case the Pi is reflashed.

### 9.1 Install librealsense from source

No matching arm64 apt binary exists for the version we need.

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.57.7
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DFORCE_RSUSB_BACKEND=ON
make -j2
sudo make install
sudo ldconfig
```

> `make -j2` keeps the Pi from overheating. Build takes ~45 minutes.

### 9.2 System dependencies

```bash
sudo apt install -y \
  libusb-1.0-0-dev libudev-dev libssl-dev \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  python3-dev python3-pip cmake build-essential git pkg-config \
  v4l-utils libv4l-dev \
  ros-humble-web-video-server
```

### 9.3 Build the workspace

```bash
mkdir -p ~/ur5e_hande_ws/src
cd ~/ur5e_hande_ws/src
git clone -b raspberry-pi https://github.com/aamhegazy/Ros2_ur5e_hande.git
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

cd ~/ur5e_hande_ws
source /opt/ros/humble/setup.bash
sudo rosdep init      # only once
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "moveit_ros_planning_interface"
colcon build --packages-select ur5e_moveit_actions_pi ros_tcp_endpoint
source install/setup.bash
```

### 9.4 Apply the TCP-Endpoint patch

See §6.

### 9.5 (Optional) realsense-ros — only if you want to compare

Known unstable on the Pi 4 + D435i pair. Build it only if you want to confirm the issue yourself; otherwise use `rpi_realsense_publisher.py` (§7).

```bash
cd ~/ur5e_hande_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ur5e_hande_ws
rosdep install -i --from-path src/realsense-ros --rosdistro humble -y
colcon build --packages-select realsense2_camera_msgs
colcon build --packages-select realsense2_camera realsense2_description
```

### 9.6 Persistent shell setup

Add to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/ur5e_hande_ws/install/setup.bash
```

---

## 10. Troubleshooting

### `Failed to resolve message name: No module named 'ur5e_moveit_actions'`
The TCP-Endpoint patch isn't applied or the symlink is missing. See §6.

### `Failed to resolve message name: No module named 'ur5e_moveit_actions.msg'`
Patch is applied but only handles 2-part names. The 3-part action handler is missing — re-apply the full patch in §6.

### `'PoseStamped' object has no attribute 'position'`
`rpi_fake_action_server.py` has a stale field path. Use `goal_handle.request.target_pose.pose.position.{x,y,z}` (note the extra `.pose`).

### Unity logs "Connection failed → Timed out after 5s" then "ROS reachable"
Cosmetic — `AppManager.cs` runs a quick TCP probe with 5s timeout that occasionally false-fails on the first attempt before retrying successfully. Ignore.

### Pi terminal shows `Connection from 172.20.10.6` then immediately `Disconnected`
First Unity probe always disconnects. The second connection (a moment later) is the real session — look for `RegisterPublisher` / `RegisterSubscriber` lines after it.

### `xioctl(UVCIOC_CTRL_QUERY) failed on control 1` from `realsense2_camera`
Known Pi 4 + D435i UVC incompatibility. Use `rpi_realsense_publisher.py` instead (§7).

### `numpy.dtype size changed` errors when running `rpi_realsense_publisher.py`
numpy 2.x is installed and incompatible with `cv_bridge`. Pin to numpy 1.x:
```bash
sudo pip3 install "numpy<2"
```

---

## Directory layout (this branch)
src/Ros2_ur5e_hande/
├── README.md                        # this file
├── ur5e_moveit_actions_pi/          # Pi-only action package
│   ├── action/
│   │   ├── PlanToPose.action
│   │   └── ExecutePlan.action
│   ├── launch/
│   │   └── rpi_bridge.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
├── ur5e_moveit_actions/src/
│   ├── rpi_fake_action_server.py
│   └── rpi_unity_action_bridge.py
├── rpi_realsense_publisher.py
└── (other packages from main branch — unused on Pi)

---

## Credits

- `ROS-TCP-Endpoint` — Unity Technologies
- `librealsense` / `pyrealsense2` — Intel RealSense
- `realsense-ros` — Intel RealSense