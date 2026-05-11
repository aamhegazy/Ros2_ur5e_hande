# UR5e + Robotiq Hand-E — ROS 2 Workspace

ROS 2 Humble workspace for controlling a **Universal Robots UR5e** arm with a **Robotiq Hand-E** gripper attached via the UR tool I/O (socat-tunneled Modbus RTU over the UR controller). Includes MoveIt 2 configuration, custom action servers, and a Unity TCP bridge for XR teleoperation.

---

## What's in here

| Package | Purpose | Source |
|---|---|---|
| `moveit_config` | MoveIt 2 configuration, launch files, SRDF/URDF assembly, ros2_control YAML | local |
| `ur5e_hande_description` | URDF/xacro + meshes for UR5e + Hand-E combined | local |
| `ur5e_moveit_actions` | Custom ROS2 action servers: `plan_to_pose` and `execute_plan` + Unity bridge | local |
| `robotiq_hande_driver` | ros2_control hardware interface for Hand-E via socat + Modbus RTU | [AGH-CEAI/robotiq_hande_driver](https://github.com/AGH-CEAI/robotiq_hande_driver) |
| `ROS-TCP-Endpoint` | Unity ↔ ROS 2 bridge (patched for action message support) | [Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) |

---

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill (desktop install)
- UR5e robot with **External Control URCap** installed on the teach pendant
- Robotiq Hand-E gripper wired to the UR tool flange (RS485 via tool I/O pins)
- A router/switch connecting the PC and the robot

---

## Install

### 1. System dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ur \
  ros-humble-ur-robot-driver \
  ros-humble-ur-description \
  ros-humble-ur-moveit-config \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-ros2-control-test-assets \
  libmodbus-dev \
  libserial-dev \
  socat
```

> `libmodbus-dev` is **mandatory** for `robotiq_hande_driver` to build.
> `ros-humble-ros2-control-test-assets` is required by `robotiq_hande_driver`.
> `socat` is required at runtime for the tool-communication tunnel.

### 2. Clone the workspace

```bash
mkdir -p ~/ur5e_hande_ws/src
cd ~/ur5e_hande_ws/src
git clone https://github.com/aamhegazy/Ros2_ur5e_hande.git .
```

### 3. Clone submodule dependencies

```bash
cd ~/ur5e_hande_ws/src
git clone -b humble-devel https://github.com/AGH-CEAI/robotiq_hande_driver.git robotiq_hande_driver
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ROS-TCP-Endpoint
```

### 4. Install ROS dependencies

```bash
cd ~/ur5e_hande_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc`:
```bash
echo "source ~/ur5e_hande_ws/install/setup.bash" >> ~/.bashrc
```

---

## Network setup

Default IPs used in this workspace:

| Device | IP |
|---|---|
| Robot | `192.168.0.10` |
| Ubuntu PC | `192.168.0.52` |
| Unity/VR PC | `192.168.0.27` |
| Subnet | `/24` on `192.168.0.0` |

### On the robot (teach pendant)

*Settings → System → Network → Static Address*
- IP: `192.168.0.10`
- Mask: `255.255.255.0`
- Gateway: `192.168.0.1`

### On the Ubuntu PC

```bash
nmcli con add type ethernet ifname <iface> con-name ur-lan \
  ipv4.method manual \
  ipv4.addresses 192.168.0.52/24
nmcli con up ur-lan
```

### Verify

```bash
ping -c 3 192.168.0.10
```

---

## Pendant setup

### External Control URCap

1. Copy the External Control `.urcap` to USB and plug into pendant
2. *Settings → System → URCaps → +* → select the file → restart
3. Create a program: *Program → URCaps → External Control*
4. Set Host IP: `192.168.0.52`, Host port: `50002`
5. Save the program

### Tool Communication Interface

Required for Hand-E via socat:

*Installation → Tool Communication Interface*:
- Enabled: **On**
- Baud: `115200`, Parity: `None`, Stop bits: `1`
- Controlled by: **User**

### Every time you launch ROS

1. Switch pendant to **Remote Control** mode
2. Load the **External Control** program
3. Start ROS first (see below), then press **▶ Play** on the pendant
4. You should see in the terminal:
   ```
   Robot connected to reverse interface. Ready to receive control commands.
   ```
5. Speed slider → **100%** (at 0% nothing moves)

> **If you relaunch ROS**, stop and re-play the URCap program on the pendant.

---

## Launch

### Real robot + Unity XR (all-in-one)

```bash
ros2 launch moveit_config spawn_ur5e_unity.launch.py \
  ur_ip:=192.168.0.10 \
  reverse_ip:=192.168.0.52 \
  ros_ip:=192.168.0.52 \
  ros_tcp_port:=10000
```

This starts:
- `ur_ros2_control_node` (UR arm + Hand-E hardware)
- `joint_state_broadcaster`
- `scaled_joint_trajectory_controller`
- `gripper_controller`
- `move_group` (MoveIt 2)
- RViz 2
- `moveit_action_server` (`/plan_to_pose`, `/execute_plan`)
- `unity_action_bridge`
- `ros_tcp_endpoint` (Unity TCP bridge on port 10000)

### Real robot only (no TCP bridge)

```bash
ros2 launch moveit_config spawn_ur5e_hande.launch.py \
  ur_ip:=192.168.0.10 \
  reverse_ip:=192.168.0.52
```

### Fake hardware (no robot needed)

```bash
ros2 launch moveit_config spawn_fake_ur5e_hande.launch.py
```

Uses `mock_components/GenericSystem` for both arm and gripper.

---

## Custom Action Servers

Two action servers are provided in `ur5e_moveit_actions`:

### `/plan_to_pose`

Plans a trajectory to a target pose using MoveIt. Returns waypoints and a `plan_id`.

```bash
ros2 action send_goal /plan_to_pose ur5e_moveit_actions/action/PlanToPose \
  "{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.2, z: 0.3}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, planning_time: 10.0, velocity_scaling: 0.1, acceleration_scaling: 0.1, planning_id: 'RRTConnect'}"
```

### `/execute_plan`

Executes a previously planned trajectory using the `plan_id` returned by `/plan_to_pose`.

```bash
ros2 action send_goal /execute_plan ur5e_moveit_actions/action/ExecutePlan \
  "{plan_id: 'YOUR_PLAN_ID_HERE'}"
```

> Use `velocity_scaling: 0.1` and `acceleration_scaling: 0.1` for safety on first runs.

---

## Unity XR Setup

On the Unity side (Windows PC at `192.168.0.27`):
- ROS IP: `192.168.0.52`
- Port: `10000`
- Protocol: ROS2

The `ROS-TCP-Endpoint` in this workspace has been patched to support 3-part action message names (e.g. `ur5e_moveit_actions/action/PlanToPose_Goal`).

---

## Kinematics calibration (recommended)

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.0.10 \
  target_filename:="$(pwd)/src/moveit_config/config/calibration/ur5e_calibration.yaml"
```

Without calibration, TCP positions can be off by ~1 cm.

---

## Troubleshooting

### Robot doesn't move after plan executes

External Control URCap is not playing. Check terminal for `Robot connected to reverse interface`. If absent, press Play on the pendant.

### `Failed to write registers (Modbus failure)` on Hand-E

- Tool Communication Interface disabled on pendant
- Wrong baud/parity (must be `115200, N, 8, 1`)
- Hand-E not receiving 24V on the tool flange
- Wrong robot IP in socat — check `Creating a virtual serial port from ip:X.X.X.X` log line

### `robotiq_hande_driver` not found by colcon

The `src/robotiq_hande_driver` folder is empty. Clone it:
```bash
cd ~/ur5e_hande_ws/src
rm -rf robotiq_hande_driver
git clone -b humble-devel https://github.com/AGH-CEAI/robotiq_hande_driver.git robotiq_hande_driver
```

### `ros2_control_test_assets` not found during build

```bash
sudo apt install -y ros-humble-ros2-control-test-assets
```

### `Neither output recipe file nor output recipe have been defined`

The recipe file paths in `ur5e_hande_description/urdf/ur5e_hande.urdf.xacro` are empty. Verify they point to:
```
/opt/ros/humble/share/ur_robot_driver/resources/ros_control.urscript
/opt/ros/humble/share/ur_robot_driver/resources/rtde_output_recipe.txt
/opt/ros/humble/share/ur_robot_driver/resources/rtde_input_recipe.txt
```

### Controllers don't activate

```bash
ros2 control list_controllers
ros2 control set_controller_state scaled_joint_trajectory_controller active
```

### Unity connection refused

Verify TCP endpoint is running and bound to the correct IP:
```
[UnityEndpoint]: Starting server on 192.168.0.52:10000
```
Check firewall: `sudo ufw allow 10000/tcp`

---

## Directory layout

```
ur5e_hande_ws/
└── src/
    ├── moveit_config/
    │   ├── config/
    │   │   ├── ur5e_hande.urdf.xacro
    │   │   ├── ur5e_hande.srdf
    │   │   ├── ros2_controllers.yaml
    │   │   ├── moveit_controllers.yaml
    │   │   ├── kinematics.yaml
    │   │   └── initial_positions.yaml
    │   └── launch/
    │       ├── spawn_ur5e_unity.launch.py       ← real robot + Unity (all-in-one)
    │       ├── spawn_ur5e_hande.launch.py        ← real robot only
    │       └── spawn_fake_ur5e_hande.launch.py   ← fake hardware
    ├── ur5e_hande_description/
    │   └── urdf/
    │       ├── ur5e_hande.urdf.xacro
    │       └── hand_e.ros2_control.xacro
    ├── ur5e_moveit_actions/                      ← custom action servers
    ├── robotiq_hande_driver/                     ← Hand-E hardware interface
    └── ROS-TCP-Endpoint/                         ← Unity bridge (patched)
```

---

## License

Each package retains its original license. See individual `LICENSE` files.

## Credits

- `robotiq_hande_driver` — AGH Centre of Excellence in Artificial Intelligence (AGH-CEAI)
- `ROS-TCP-Endpoint` — Unity Technologies
- Universal Robots ROS 2 Driver — Universal Robots A/S & PickNik Robotics