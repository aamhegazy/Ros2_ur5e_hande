# UR5e + Robotiq Hand-E — ROS 2 Humble Workspace

ROS 2 Humble workspace for controlling a **Universal Robots UR5e** arm with a **Robotiq Hand-E** gripper mounted on the UR tool flange. The gripper is driven over Modbus RTU through a socat tunnel, using the UR's built-in Tool Communication Interface — no USB-RS485 adapter needed. Includes MoveIt 2 configuration and a Unity TCP bridge for XR applications.

```
[ROS 2 on PC]  ──TCP/IP──►  [UR Controller]  ──RS485 tool I/O──►  [Hand-E]
                              └ External Control URCap forwards control
                              └ Tool Comm forwards socat tunnel to RS485
```

## Contents

| Package | Purpose | Source |
|---|---|---|
| `moveit_config` | MoveIt 2 config, launch files, URDF top-level, ros2_control YAML | local |
| `ur5e_hande_description` | URDF/xacro + meshes for UR5e + Hand-E | local |
| `robotiq_hande_driver` | ros2_control hardware interface for Hand-E (socat + Modbus RTU) | [AGH-CEAI/robotiq_hande_driver](https://github.com/AGH-CEAI/robotiq_hande_driver) |
| `ROS-TCP-Endpoint` | Unity ↔ ROS 2 bridge for XR teleop | [Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) |

## Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill (desktop install)
- UR5e with the **External Control URCap** installed on the teach pendant
- Robotiq Hand-E wired to the UR tool flange (24V + RS485 on tool I/O pins)
- Ethernet connection between PC and robot

---

## 1. Install

### System dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-ur \
  ros-humble-ur-robot-driver \
  ros-humble-ur-description \
  ros-humble-ur-moveit-config \
  ros-humble-ur-calibration \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  libmodbus-dev \
  libserial-dev \
  socat
```

`libmodbus-dev` is **required** for the Hand-E driver build. `socat` is required at runtime for the tool-communication tunnel.

### Create the workspace

```bash
mkdir -p ~/ur5e_hande_ws/src
cd ~/ur5e_hande_ws/src
# Unzip this archive here so you end up with:
#   ~/ur5e_hande_ws/src/moveit_config
#   ~/ur5e_hande_ws/src/ur5e_hande_description
#   ~/ur5e_hande_ws/src/robotiq_hande_driver
#   ~/ur5e_hande_ws/src/ROS-TCP-Endpoint
```

### Resolve ROS dependencies

```bash
cd ~/ur5e_hande_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --symlink-install
source install/setup.bash
```

Consider adding the source line to `~/.bashrc`:

```bash
echo "source ~/ur5e_hande_ws/install/setup.bash" >> ~/.bashrc
```

---

## 2. Network setup

This workspace defaults to:

| Device | IP |
|---|---|
| Robot | `192.168.0.10` |
| PC | `192.168.0.11` |

### On the pendant

*Settings → System → Network → Static Address*
- IP: `192.168.0.10`
- Subnet mask: `255.255.255.0`
- Gateway: `192.168.0.1`

### On the PC

Find your wired interface:

```bash
ip -br link
```

Replace `<iface>` with the name you found (e.g. `enp3s0`, `enx6c1ff71e5bcf`):

```bash
nmcli con add type ethernet ifname <iface> con-name ur-lan \
  ipv4.method manual \
  ipv4.addresses 192.168.0.11/24 \
  ipv4.dns 192.168.0.1
nmcli con up ur-lan
```

No `ipv4.gateway` — internet stays on Wi-Fi.

### Verify

```bash
ping -c 3 192.168.0.10    # PC → robot
```

From the pendant, also ping back to `192.168.0.11`. Both directions must succeed.

### Using different IPs

Pass them at launch time:

```bash
ros2 launch moveit_config spawn_ur5e_hande.launch.py \
  ur_ip:=10.0.0.10 reverse_ip:=10.0.0.11
```

---

## 3. Pendant setup

Do this **once** per robot.

### Install the External Control URCap

1. Copy the `.urcap` file to a USB stick, plug into the pendant.
2. *Settings → System → URCaps → +* → select the file → restart.
3. Create a program: *Program → URCaps → External Control*.
4. Set **Host IP = `192.168.0.11`** (your PC), **Host port = `50002`**.
5. Save the program.

### Enable Tool Communication Interface

This is required for the Hand-E — the gripper is driven over RS485 through the tool I/O.

*Installation → URCaps → Tool Communication Interface* (or *Tool I/O* in some PolyScope versions):

- **Enabled**: On
- **Controlled by**: User
- Baud: `115200`
- Parity: `None`
- Stop bits: `1`
- RX idle chars: `1.5`
- TX idle chars: `3.5`

Save the installation.

---

## 4. Kinematics calibration

Each UR5e ships with factory-calibrated DH offsets unique to that specific arm. Without them, TCP positions can be off by up to ~1 cm depending on joint angle — critical for Cartesian teleop or pick-and-place.

Extract the calibration from your robot:

```bash
cd ~/ur5e_hande_ws
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.0.10 \
  target_filename:="$(pwd)/src/moveit_config/config/calibration/ur5e_calibration.yaml"
```

Then rebuild:

```bash
colcon build --symlink-install --packages-select moveit_config
source install/setup.bash
```

The launch file automatically uses this file. If it's missing, the launch falls back to UR's generic defaults and prints a warning.

---

## 5. Launch

### Every-time checklist

1. **PC is on the right subnet** (`192.168.0.11/24`). Check with `ip addr show`.
2. **Pendant in Remote Control mode** — top-right icon on PolyScope must show Remote, not Local.
3. **Speed slider at 100%** — top of the pendant screen.
4. **No E-stop pressed**.
5. **Load the External Control program** on the pendant (but don't press Play yet).

### Start ROS

```bash
ros2 launch moveit_config spawn_ur5e_hande.launch.py
```

This brings up:

- UR hardware interface + Hand-E hardware interface
- socat tunnel (`/tmp/ttyUR` ↔ `192.168.0.10:54321`)
- Controllers: joint state broadcaster, `scaled_joint_trajectory_controller` (arm), `gripper_controller` (Hand-E)
- MoveIt MoveGroup
- RViz with MotionPlanning panel

### Press Play on the pendant

Once the driver prints:

```
RobotiqHandeHardwareInterface: Connected
Waiting on fulfillment of program request
```

Press **▶ Play** on the pendant. The driver should then print:

```
Robot requested program
Sent program to robot
Robot connected to reverse interface. Ready to receive control commands.
```

**Only after this message** can the arm actually move. Before it, trajectory goals will "succeed" with no physical motion.

If you kill ROS and relaunch, you must stop the program on the pendant and press Play again to re-establish the reverse interface.

### Fake hardware (no robot needed)

```bash
ros2 launch moveit_config spawn_fake_ur5e_hande.launch.py
```

Uses mock components for both arm and gripper. Good for testing launch files, URDF, and MoveIt config without the physical robot.

### Unity XR bridge

Start the TCP bridge in a separate terminal after the main launch is running:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

Configure the Unity side to connect to your PC's IP on port `10000`.

---

## 6. Controlling the robot

### MoveIt via RViz

In the MotionPlanning panel's Planning tab:
- Drag the interactive marker on the end-effector.
- Click **Plan** to preview, then **Execute**.
- Or **Plan & Execute** to do both.

Switch between planning groups `ur5e` (arm) and `gripper` using the Planning Group dropdown.

### Direct trajectory commands (no MoveIt)

```bash
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
     joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
     points: [{positions: [0, -1.57, 0, -1.57, 0, 0], time_from_start: {sec: 5}}]}}"
```

All angles in radians. 1.57 rad ≈ 90°.

### Gripper

```bash
# Close
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 10.0}}"

# Open
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.025, max_effort: 10.0}}"
```

Position range: `0.0` m (fully closed) to `0.025` m (fully open). Max effort in Newtons.

### Inspect current state

```bash
ros2 topic echo /joint_states           # all joint positions/velocities
ros2 control list_controllers           # which controllers are active
ros2 topic echo /tcp_pose_broadcaster/pose  # current TCP pose
```

---

## 7. Troubleshooting

### "Goal reached, success!" but the arm doesn't move

The External Control URCap isn't running on the pendant. The ROS controller accepts the goal, runs its internal trajectory timer, and reports success — but the commands went into a black hole because the reverse interface isn't connected.

**Check**: did the driver print *"Robot connected to reverse interface. Ready to receive control commands"* when you launched? If not, press Play on the pendant.

### `Failed to write registers (Modbus failure)` on Hand-E activation

One of:
- Tool Communication Interface is **not enabled** on the pendant installation.
- Baud/parity mismatch between driver and pendant. Must be `115200, N, 8, 1` on both sides.
- Hand-E not powered (no 24V on tool flange).
- Wrong IP reaching socat. Check this log line: `Creating a virtual serial port from ip:X.X.X.X port:54321 with socat`. The IP must match your robot.

### `package 'robotiq_hande_description' not found`

Old reference left over from an earlier version. The current xacro points at `ur5e_hande_description/urdf/hand_e.ros2_control.xacro`. Make sure your `moveit_config/config/ur5e_hande.urdf.xacro` matches what's in this repo.

### `A required package was not found ... pkg_check_modules`

Missing `libmodbus-dev` — the Hand-E driver needs it:

```bash
sudo apt install libmodbus-dev libserial-dev
cd ~/ur5e_hande_ws
rm -rf build/robotiq_hande_driver install/robotiq_hande_driver
colcon build --symlink-install
```

### Arm trajectory rejected: "start position differs from current"

Current pose is too far from the trajectory's first waypoint. Options:
- Use Freedrive on the pendant to move closer.
- Send a trajectory whose first waypoint equals the current state (check `/joint_states`).
- Use MoveIt instead of raw trajectories — MoveIt handles start-state automatically.

### `/tmp/ttyUR` permission denied

```bash
sudo usermod -aG dialout $USER
# then log out and back in
```

### Static IP keeps getting overridden by DHCP

```bash
nmcli con show
```

Delete any other active connection on the same interface, or raise `connection.autoconnect-priority` on the `ur-lan` profile.

---

## Directory layout

```
src/
├── README.md                        # this file
├── .gitignore
├── moveit_config/                   # MoveIt config + launch files (edit here for integration)
│   ├── config/
│   │   ├── calibration/
│   │   │   └── ur5e_calibration.yaml     # generated per-robot (see §4)
│   │   ├── ur5e_hande.urdf.xacro         # top-level URDF assembly
│   │   ├── ur5e_hande.srdf
│   │   ├── ros2_controllers.yaml
│   │   ├── moveit_controllers.yaml
│   │   └── ...
│   └── launch/
│       ├── spawn_ur5e_hande.launch.py       # real hardware
│       └── spawn_fake_ur5e_hande.launch.py  # mock hardware
├── ur5e_hande_description/          # URDF + meshes for UR5e + Hand-E
├── robotiq_hande_driver/            # Hand-E ros2_control plugin (upstream)
└── ROS-TCP-Endpoint/                # Unity bridge (upstream)
```

## Credits

- `robotiq_hande_driver` — AGH University of Krakow Centre of Excellence in AI
- `ROS-TCP-Endpoint` — Unity Technologies
- Universal Robots ROS 2 Driver — Universal Robots A/S & PickNik Robotics

Each package retains its original license. See individual `LICENSE` files.

---

## 8. Raspberry Pi 4 — RealSense Bridge (raspberry-pi branch)

The `raspberry-pi` branch adds a Raspberry Pi 4 as a lightweight ROS 2 bridge between Unity and the main Ubuntu PC planning stack. The Pi runs the ROS-TCP-Endpoint (Unity TCP bridge) and the Intel RealSense D435i camera node.

### Network layout

| Device | IP | Role |
|---|---|---|
| Mobile Hotspot | `172.20.10.1` | Gateway |
| Ubuntu PC | `172.20.10.4` | Full ROS 2 planning stack |
| Raspberry Pi 4 | `172.20.10.2` | Unity TCP bridge + RealSense |
| Windows PC (Unity) | `172.20.10.6` | XR frontend |

SSH into the Pi:
```bash
ssh hgz-rpi4@172.20.10.2
```

### Pi workspace layout
### 1. Build librealsense from source

The Pi requires building librealsense from source (no apt binary for arm64 matching the required version).

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

> **Note:** `make -j2` keeps the Pi from overheating. Build takes ~45 minutes.

### 2. Install system dependencies

```bash
sudo apt install -y \
  libusb-1.0-0-dev libudev-dev libssl-dev \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  python3-dev python3-pip cmake build-essential git pkg-config \
  v4l-utils libv4l-dev
```

### 3. Build realsense-ros

```bash
cd ~/ur5e_hande_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ur5e_hande_ws
source /opt/ros/humble/setup.bash
sudo rosdep init    # only needed once
rosdep update
rosdep install -i --from-path src/realsense-ros --rosdistro humble -y
colcon build --packages-select realsense2_camera_msgs
colcon build --packages-select realsense2_camera realsense2_description
```

> Build order matters — `realsense2_camera_msgs` must be built first.

### 4. Launch RealSense camera

Always plug the D435i into a **USB 3.0 port** (blue) on the Pi. USB 2.x causes timeout errors and degraded performance.

```bash
source /opt/ros/humble/setup.bash
source ~/ur5e_hande_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

With point cloud enabled:
```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Verify topics:
```bash
ros2 topic list
# Expected:
# /camera/camera/color/image_raw
# /camera/camera/color/camera_info
# /camera/camera/depth/image_rect_raw
# /camera/camera/depth/camera_info
# /camera/camera/depth/color/points  (if pointcloud.enable:=true)
```

Check framerate:
```bash
ros2 topic hz /camera/camera/color/image_raw   # ~30 Hz on USB 3.0
```

> The `Mipi device capability` permission errors in the log are harmless on Pi — they are MIPI interface probes that always fail on USB-connected devices.

### 5. Launch Unity TCP bridge

```bash
source /opt/ros/humble/setup.bash
source ~/ur5e_hande_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=172.20.10.2 -p ROS_TCP_PORT:=10000
```

Unity settings: **ROS IP = `172.20.10.2`**, **Port = `10000`**, **Protocol = ROS2**.

### 6. ROS-TCP-Endpoint patch

The default v0.7.0 does not support action message types. A patch was applied to `server.py` → `resolve_message_name()` to handle 3-part message names like `ur5e_moveit_actions/action/PlanToPose_Goal` and resolve action inner classes (`PlanToPose.Goal`, etc.).

### Unity topics (Pi bridge)

| Topic | Direction | Type |
|---|---|---|
| `plan_to_pose/goal` | Unity → Pi → Ubuntu | `ur5e_moveit_actions/action/PlanToPose_Goal` |
| `plan_to_pose/result` | Ubuntu → Pi → Unity | `ur5e_moveit_actions/action/PlanToPose_Result` |
| `execute_plan/goal` | Unity → Pi → Ubuntu | `ur5e_moveit_actions/action/ExecutePlan_Goal` |
| `execute_plan/result` | Ubuntu → Pi → Unity | `ur5e_moveit_actions/action/ExecutePlan_Result` |

### 7. Custom RealSense ROS 2 Publisher (realsense_publisher.py)

Due to a known incompatibility between `realsense-ros` and the Pi 4 (UVC extension unit `xioctl` timeouts on control 1), the standard `rs_launch.py` fails to start the device. The workaround is `realsense_publisher.py` — a Python node that uses `pyrealsense2` directly and publishes to ROS 2 topics manually.

**Topics published:**
| Topic | Type |
|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` |

**Run:**
```bash
source /opt/ros/humble/setup.bash
python3 ~/ur5e_hande_ws/src/Ros2_ur5e_hande/realsense_publisher.py
```

**View stream in browser (no display needed):**
```bash
# Terminal 2
ros2 run web_video_server web_video_server
```
Then open: `http://172.20.10.2:8080/stream?topic=/camera/color/image_raw`

**Notes:**
- Requires `numpy<2` — ROS 2 cv_bridge is incompatible with numpy 2.x
- Camera must be on USB 3.0 port (blue) — USB 2.x causes bandwidth issues
- `pyrealsense2` must be installed system-wide: `sudo pip3 install pyrealsense2 "numpy<2"`
