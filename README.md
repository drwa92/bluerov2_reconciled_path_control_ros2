# BlueROV2 Reconciled Path Control for ROS 2 / DAVE

A clean ROS 2 direct-thruster path-following controller for the BlueROV2 in DAVE/Gazebo.

Maintained by **Waseem Akram** at **[Labust LAB](https://github.com/labust)**.

This repository provides service-based motion commands for:

- go-to pose control;
- waypoint following;
- circle following;
- spiral following;
- generic trajectory following;
- stop and hold;
- emergency stop;
- optional model-aided virtual-wrench reconciliation for research use.

The package is designed for the **DAVE ROS 2 BlueROV2 simulator** using direct thruster commands. It is intended to be simple enough for normal path-following experiments, while also providing optional modules for advanced fault-tolerant control and sensor-actuator reconciliation research.

---

## 1. Prerequisites

This package requires a working installation of the **DAVE ROS 2 BlueROV2 simulator**.

Please install and verify DAVE before using this controller:

<https://dave-ros2.notion.site/?v=d54cc8422868455888cc629d8e6117a9>

The controller assumes that DAVE provides the BlueROV2 odometry topic and the six Gazebo thruster command topics:

```text
/model/bluerov2/odometry
/model/bluerov2/joint/thruster1_joint/cmd_thrust
/model/bluerov2/joint/thruster2_joint/cmd_thrust
/model/bluerov2/joint/thruster3_joint/cmd_thrust
/model/bluerov2/joint/thruster4_joint/cmd_thrust
/model/bluerov2/joint/thruster5_joint/cmd_thrust
/model/bluerov2/joint/thruster6_joint/cmd_thrust
```

Required software:

- ROS 2 Jazzy or a compatible ROS 2 distribution;
- DAVE ROS 2 BlueROV2 simulator;
- Gazebo / Gazebo Harmonic transport tools used by DAVE;
- `ros_gz_bridge`;
- Python 3;
- `numpy`;
- standard ROS 2 message packages: `geometry_msgs`, `nav_msgs`, `std_msgs`, `std_srvs`.

> **Important:** This package uses direct thruster-level control. For direct allocation experiments, launch DAVE with `use_ardusub:=false` and `use_teleop:=false` so that ArduSub/MAVROS does not compete with this controller for the thruster command topics.

---

## 2. Package structure

```text
bluerov2_reconciled_path_control_ros2/
├── src/
│   ├── bluerov2_path_interfaces/
│   │   └── srv/
│   │       ├── GoTo.srv
│   │       ├── FollowWaypoints.srv
│   │       ├── FollowCircle.srv
│   │       ├── FollowSpiral.srv
│   │       └── FollowTrajectory.srv
│   │
│   └── bluerov2_path_control/
│       ├── bluerov2_path_control/
│       │   ├── path_controller_node.py
│       │   ├── mission_client.py
│       │   ├── hydrodynamics.py
│       │   └── reconciliation.py
│       │
│       ├── launch/
│       │   ├── path_controller.launch.py
│       │   └── direct_thruster_bridge.launch.py
│       │
│       └── config/
│           └── bluerov2_path_control.yaml
│
├── docs/
├── README.md
├── LICENSE
└── CITATION.cff
```

---

## 3. Main features

### 3.1 Mission-level services

The controller exposes the following ROS 2 services:

```text
/path_controller/go_to
/path_controller/follow_waypoints
/path_controller/follow_circle
/path_controller/follow_spiral
/path_controller/follow_trajectory
/path_controller/stop
/path_controller/emergency_stop
/path_controller/enable_reconciliation
/path_controller/reset_reconciliation
```

### 3.2 Direct-thruster allocation

The controller subscribes to DAVE odometry and publishes directly to the six BlueROV2 thruster command topics:

```text
/model/bluerov2/joint/thruster1_joint/cmd_thrust
/model/bluerov2/joint/thruster2_joint/cmd_thrust
/model/bluerov2/joint/thruster3_joint/cmd_thrust
/model/bluerov2/joint/thruster4_joint/cmd_thrust
/model/bluerov2/joint/thruster5_joint/cmd_thrust
/model/bluerov2/joint/thruster6_joint/cmd_thrust
```

The package uses experimentally identified DAVE BlueROV2 thrust-allocation signs:

```text
T1: -X, -Y, -yaw
T2: -X, +Y, +yaw
T3: +X, -Y, +yaw
T4: +X, +Y, -yaw
T5: -Z
T6: -Z
```

The resulting allocation map for the reduced wrench vector

```text
tau = [X, Y, Z, N]^T
```

is

```text
B = [
  [-0.707, -0.707,  0.707,  0.707,  0.0,  0.0],
  [-0.707,  0.707, -0.707,  0.707,  0.0,  0.0],
  [ 0.0,    0.0,    0.0,    0.0,   -1.0, -1.0],
  [-1.0,    1.0,    1.0,   -1.0,    0.0,  0.0]
]
```

The allocator enforces both magnitude and rate constraints:

```text
u_min <= u_k <= u_max
|u_k - u_{k-1}| <= du_max
```

---

## 4. Optional research module: virtual-wrench reconciliation

This repository includes optional advanced modules for research:

```text
hydrodynamics.py
reconciliation.py
```

The reconciliation module is disabled by default so that the package behaves as a clean path-following controller for normal use.

### 4.1 Model-aided wrench reconstruction

The hydrodynamics module provides BlueROV2 heavy/classic parameter sets and estimates a reduced 4-DOF generalized wrench:

```text
tau_hat = [X_hat, Y_hat, Z_hat, N_hat]^T
```

from body velocity and acceleration using a simplified model:

```text
tau_hat = M_eff * nu_dot + D(nu) * nu
```

where

```text
nu = [u, v, w, r]^T.
```

### 4.2 Projected virtual-wrench reconciliation

The reconciliation module estimates a bounded net actuator-induced wrench defect:

```text
r_a_hat ≈ tau_actual - B u
```

The projected update has the form:

```text
e_a = tau_hat - B u - r_a_hat
r_a_hat_next = Proj_R(r_a_hat + ell_a * e_a)
```

The allocator can then compensate using:

```text
tau_target = tau_cmd - r_a_hat
```

This module can be enabled at runtime using a ROS 2 service.

---

## 5. Installation

### 5.1 Install DAVE first

Install the DAVE ROS 2 BlueROV2 simulator by following the official setup instructions:

<https://dave-ros2.notion.site/?v=d54cc8422868455888cc629d8e6117a9>

Verify that DAVE launches correctly before building this controller package.

### 5.2 Clone this repository

Option A: clone into the ROS 2 workspace source folder:

```bash
cd ~/dave_ws/src
git clone https://github.com/drwa92/bluerov2_reconciled_path_control_ros2.git
```

Because this repository contains packages under its own `src/` directory, `colcon` will discover them recursively.

Option B: keep only the two packages directly under `~/dave_ws/src`:

```text
~/dave_ws/src/bluerov2_path_interfaces
~/dave_ws/src/bluerov2_path_control
```

Both layouts are acceptable as long as duplicate copies of the same packages are not present in the workspace.

### 5.3 Build

```bash
cd ~/dave_ws
colcon build --packages-select bluerov2_path_interfaces bluerov2_path_control
source install/setup.bash
```

### 5.4 Verify installation

```bash
ros2 pkg list | grep bluerov2_path
```

Expected:

```text
bluerov2_path_control
bluerov2_path_interfaces
```

Check custom interfaces:

```bash
ros2 interface list | grep bluerov2_path_interfaces
```

Expected:

```text
bluerov2_path_interfaces/srv/FollowCircle
bluerov2_path_interfaces/srv/FollowSpiral
bluerov2_path_interfaces/srv/FollowTrajectory
bluerov2_path_interfaces/srv/FollowWaypoints
bluerov2_path_interfaces/srv/GoTo
```

Check executables:

```bash
ros2 pkg executables bluerov2_path_control
```

Expected:

```text
bluerov2_path_control mission_client
bluerov2_path_control path_controller
```

---

## 6. Launching

### Terminal 1: launch DAVE in direct-control mode

```bash
cd ~/dave_ws
source install/setup.bash

ros2 launch dave_demos dave_robot.launch.py \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false \
  use_ardusub:=false \
  use_teleop:=false
```

This launch mode is recommended because this package directly commands thruster topics. If ArduSub/teleop is enabled, it may also publish to the internal thruster topics and compete with this controller.

### Terminal 2: launch controller and bridge

```bash
cd ~/dave_ws
source install/setup.bash

ros2 launch bluerov2_path_control path_controller.launch.py \
  model_name:=bluerov2 \
  use_bridge:=true
```

The launch file starts the path controller and bridges the six Gazebo thruster command topics to ROS 2.

### Terminal 3: check services

```bash
ros2 service list | grep path_controller
```

Expected:

```text
/path_controller/go_to
/path_controller/follow_waypoints
/path_controller/follow_circle
/path_controller/follow_spiral
/path_controller/follow_trajectory
/path_controller/stop
/path_controller/emergency_stop
/path_controller/enable_reconciliation
/path_controller/reset_reconciliation
```

---

## 7. Usage examples

### 7.1 Go to one pose

```bash
ros2 service call /path_controller/go_to bluerov2_path_interfaces/srv/GoTo \
"{x: 0.0, y: 0.0, z: -0.5, yaw: 0.0, speed: 0.15, hold_at_goal: true}"
```

The ROV will move to the requested pose and hold there.

### 7.2 Follow waypoints

```bash
ros2 service call /path_controller/follow_waypoints bluerov2_path_interfaces/srv/FollowWaypoints \
"{waypoints: [
  {position: {x: 1.0, y: 0.0, z: -0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
  {position: {x: 1.0, y: 1.0, z: -0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
  {position: {x: 0.0, y: 1.0, z: -0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
  {position: {x: 0.0, y: 0.0, z: -0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
], speed: 0.12, loop: false, hold_at_end: true}"
```

### 7.3 Follow a circle

Fixed yaw:

```bash
ros2 service call /path_controller/follow_circle bluerov2_path_interfaces/srv/FollowCircle \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius: 1.0, period: 90.0, turns: 1.0, clockwise: false, yaw_mode: 'fixed', yaw: 0.0, hold_at_end: true}"
```

Tangent yaw:

```bash
ros2 service call /path_controller/follow_circle bluerov2_path_interfaces/srv/FollowCircle \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius: 1.0, period: 90.0, turns: 1.0, clockwise: false, yaw_mode: 'tangent', yaw: 0.0, hold_at_end: true}"
```

Use `turns: 0.0` for continuous circle tracking until stopped.

### 7.4 Follow a spiral

```bash
ros2 service call /path_controller/follow_spiral bluerov2_path_interfaces/srv/FollowSpiral \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius_start: 0.1, radius_end: 1.8, duration: 160.0, turns: 2.0, clockwise: false, yaw_mode: 'tangent', yaw: 0.0, hold_at_end: true}"
```

### 7.5 Generic trajectory service

Square:

```bash
ros2 service call /path_controller/follow_trajectory bluerov2_path_interfaces/srv/FollowTrajectory \
"{trajectory_type: 'square', params: [2.0, 30.0, -0.5, 0.0], yaw_mode: 'fixed', hold_at_end: true}"
```

Circle:

```bash
ros2 service call /path_controller/follow_trajectory bluerov2_path_interfaces/srv/FollowTrajectory \
"{trajectory_type: 'circle', params: [0.0, 0.0, -0.5, 1.0, 90.0, 1.0, 0.0, 0.0], yaw_mode: 'tangent', hold_at_end: true}"
```

Spiral:

```bash
ros2 service call /path_controller/follow_trajectory bluerov2_path_interfaces/srv/FollowTrajectory \
"{trajectory_type: 'spiral', params: [0.0, 0.0, -0.5, 0.1, 1.8, 160.0, 2.0, 0.0, 0.0], yaw_mode: 'tangent', hold_at_end: true}"
```

---

## 8. Stop and emergency stop

### 8.1 Soft stop

Stops the current mission and holds the current pose.

```bash
ros2 service call /path_controller/stop std_srvs/srv/Trigger "{}"
```

### 8.2 Emergency stop

Immediately sends zero thrust and disables controller output.

```bash
ros2 service call /path_controller/emergency_stop std_srvs/srv/SetBool "{data: true}"
```

Clear emergency stop:

```bash
ros2 service call /path_controller/emergency_stop std_srvs/srv/SetBool "{data: false}"
```

After clearing emergency stop, the controller holds the current pose.

---

## 9. CLI mission helper

The package includes a small CLI helper:

```bash
ros2 run bluerov2_path_control mission_client --help
```

### Go to

```bash
ros2 run bluerov2_path_control mission_client goto \
  --x 1.0 --y 0.0 --z -0.5 --yaw 0.0 --speed 0.15
```

### Circle

```bash
ros2 run bluerov2_path_control mission_client circle \
  --cx 0.0 --cy 0.0 --z -0.5 \
  --radius 1.0 --period 90.0 --turns 1.0 \
  --yaw-mode tangent
```

### Spiral

```bash
ros2 run bluerov2_path_control mission_client spiral \
  --cx 0.0 --cy 0.0 --z -0.5 \
  --r0 0.1 --r1 1.8 \
  --duration 160.0 --turns 2.0 \
  --yaw-mode tangent
```

### Stop

```bash
ros2 run bluerov2_path_control mission_client stop
```

### Emergency stop

```bash
ros2 run bluerov2_path_control mission_client estop --on
```

Clear emergency stop:

```bash
ros2 run bluerov2_path_control mission_client estop --off
```

---

## 10. Enabling virtual-wrench reconciliation

The reconciliation module is disabled by default.

Enable it:

```bash
ros2 service call /path_controller/enable_reconciliation std_srvs/srv/SetBool "{data: true}"
```

Disable it:

```bash
ros2 service call /path_controller/enable_reconciliation std_srvs/srv/SetBool "{data: false}"
```

Reset the estimate:

```bash
ros2 service call /path_controller/reset_reconciliation std_srvs/srv/Trigger "{}"
```

### Diagnostics

```bash
ros2 topic list | grep reconciliation
```

Expected topics:

```text
/path_controller/reconciliation/estimated_wrench
/path_controller/reconciliation/nominal_wrench
/path_controller/reconciliation/raw_residual
/path_controller/reconciliation/compensated_residual
/path_controller/reconciliation/rhat
```

Echo the estimated virtual-wrench correction:

```bash
ros2 topic echo /path_controller/reconciliation/rhat
```

Echo raw and compensated residuals:

```bash
ros2 topic echo /path_controller/reconciliation/raw_residual
ros2 topic echo /path_controller/reconciliation/compensated_residual
```

---

## 11. Configuration

Main parameters are in:

```text
bluerov2_path_control/config/bluerov2_path_control.yaml
```

Important parameters include:

```yaml
controller_mode: pid        # pid or smc
thruster_max: 40.0
thruster_rate_max: 80.0
kp_xy: 8.0
kd_xy: 8.0
kp_z: 35.0
kd_z: 12.0
ki_z: 6.0
kp_yaw: 2.0
kd_yaw: 1.5

reconciliation:
  enable_wrench_reconciliation: false
  model_name: heavy
  recon_gain: 0.08
  rhat_bound_xy: 8.0
  rhat_bound_z: 12.0
  rhat_bound_yaw: 8.0
  rhat_rate_limit: 0.8
```

---

## 12. Recommended test sequence

### Test 1: depth hold

```bash
ros2 service call /path_controller/go_to bluerov2_path_interfaces/srv/GoTo \
"{x: 0.0, y: 0.0, z: -0.5, yaw: 0.0, speed: 0.15, hold_at_goal: true}"
```

### Test 2: small circle

```bash
ros2 service call /path_controller/follow_circle bluerov2_path_interfaces/srv/FollowCircle \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius: 0.8, period: 100.0, turns: 1.0, clockwise: false, yaw_mode: 'fixed', yaw: 0.0, hold_at_end: true}"
```

### Test 3: tangent circle

```bash
ros2 service call /path_controller/follow_circle bluerov2_path_interfaces/srv/FollowCircle \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius: 1.0, period: 90.0, turns: 1.0, clockwise: false, yaw_mode: 'tangent', yaw: 0.0, hold_at_end: true}"
```

### Test 4: spiral

```bash
ros2 service call /path_controller/follow_spiral bluerov2_path_interfaces/srv/FollowSpiral \
"{center_x: 0.0, center_y: 0.0, z: -0.5, radius_start: 0.1, radius_end: 1.5, duration: 140.0, turns: 2.0, clockwise: false, yaw_mode: 'tangent', yaw: 0.0, hold_at_end: true}"
```

---

## 13. Troubleshooting

### 13.1 DAVE is not installed or the simulator does not launch

Install and verify the DAVE ROS 2 BlueROV2 simulator before using this package:

<https://dave-ros2.notion.site/?v=d54cc8422868455888cc629d8e6117a9>

A typical direct-control launch command is:

```bash
ros2 launch dave_demos dave_robot.launch.py \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false \
  use_ardusub:=false \
  use_teleop:=false
```

### 13.2 Duplicate package names during build

If you see:

```text
Duplicate package names not supported
```

make sure you did not unzip a full workspace inside `~/dave_ws` while also copying its packages into `~/dave_ws/src`.

Fix:

```bash
cd ~/dave_ws
rm -rf bluerov2_adv_ws bluerov2_clean_control_ws bluerov2_clean_control_ws_fixed
touch backups/COLCON_IGNORE 2>/dev/null || true
```

If your GitHub repository folder is inside `~/dave_ws`, either move it outside the workspace or add:

```bash
touch ~/dave_ws/bluerov2_reconciled_path_control_ros2/COLCON_IGNORE
```

Then rebuild:

```bash
colcon build --packages-select bluerov2_path_interfaces bluerov2_path_control
source install/setup.bash
```

### 13.3 Interface package not found

Check:

```bash
ros2 interface list | grep bluerov2_path_interfaces
```

If empty, rebuild cleanly:

```bash
cd ~/dave_ws
rm -rf build/bluerov2_path_interfaces install/bluerov2_path_interfaces
rm -rf build/bluerov2_path_control install/bluerov2_path_control
colcon build --packages-select bluerov2_path_interfaces bluerov2_path_control
source install/setup.bash
```

### 13.4 Thruster command topics do not exist

Confirm Gazebo thruster topics exist:

```bash
gz topic -l | grep cmd_thrust
```

Expected:

```text
/model/bluerov2/joint/thruster1_joint/cmd_thrust
/model/bluerov2/joint/thruster2_joint/cmd_thrust
/model/bluerov2/joint/thruster3_joint/cmd_thrust
/model/bluerov2/joint/thruster4_joint/cmd_thrust
/model/bluerov2/joint/thruster5_joint/cmd_thrust
/model/bluerov2/joint/thruster6_joint/cmd_thrust
```

If these topics are missing, DAVE BlueROV2 is not launched correctly. Revisit the DAVE setup instructions:

<https://dave-ros2.notion.site/?v=d54cc8422868455888cc629d8e6117a9>

### 13.5 Thruster commands do not move the ROV

Confirm ROS bridge topics exist:

```bash
ros2 topic list | grep cmd_thrust
```

If using direct control, make sure DAVE was launched with:

```bash
use_ardusub:=false use_teleop:=false
```

### 13.6 ROV moves in the wrong direction

The allocation matrix may not match your vehicle model variant. Run a single-thruster sign test and update the allocation signs if needed.

### 13.7 Controller oscillates

Reduce gains:

```yaml
kp_xy: lower
kd_xy: slightly higher
kp_yaw: lower
```

### 13.8 ROV lags on moving paths

Make sure reference-velocity feedforward is enabled in the configuration.

---

## 14. Notes for research use

This package is a clean path-control base. For journal experiments, the optional reconciliation module can be extended with:

- sensor-fault injection;
- sensor reconciliation using multiple ROS topics;
- actuator-fault injection;
- projected sensor-gated virtual-wrench reconciliation;
- comparison against nominal RateQP and residual-health allocation.

The core research idea is:

```text
sensor residuals       -> sensor reconciliation
rate/allocation error  -> rate-aware allocation
actuator wrench error  -> projected virtual-wrench reconciliation
```

---

## 15. Safety warning

This package directly commands thruster force topics. Use it first in simulation. Do not run it on hardware without adding hardware safety checks, watchdogs, command limits, arming logic, and operator supervision.

---

## 16. License

MIT License.

---

## 17. Citation

If you use this package in academic work, please cite the associated paper once available.

Suggested placeholder:

```bibtex
@misc{akram_bluerov2_reconciled_path_control,
  author = {Akram, Waseem},
  title = {BlueROV2 Reconciled Path Control for ROS 2 and DAVE},
  year = {2026},
  note = {ROS 2 package for direct-thruster BlueROV2 path following and virtual-wrench reconciliation}
}
```

---



## Affiliation

This package is maintained by **Waseem Akram** at **Labust**.

- GitHub: <https://github.com/drwa92>
- Contact: <drwa92@gmail.com>
