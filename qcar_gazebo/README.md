# qcar_gazebo

ROS 2 / Gazebo Classic simulation, SLAM, and Nav2 (MPPI + SmacPlannerHybrid) navigation stack
for the QCar Ackermann-steered robot. This package drives **both** the Gazebo simulation and the
real QCar - the same Nav2 configuration, planner, controller, and custom plugins run in both
cases; see the sibling [`qcar_hardware`](../qcar_hardware) package for the onboard bridge that
makes the real robot look like a normal ROS2 robot to this package.

## Prerequisites

- **ROS 2 Humble on Ubuntu 22.04 (Jammy)**, or another combination where Gazebo **classic**
  (`gazebo11`) is still packaged. This stack uses the classic `gazebo_ros`/`gazebo_plugins`
  plugins (`libgazebo_ros_factory.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`,
  `libgazebo_ros_ackermann_drive.so`) and the `gazebo`/`spawn_entity.py` executables - none of
  which exist for newer distros/OSes that ship only the new Gazebo (Harmonic/Ionic, `gz-sim`).
  ROS 2 Jazzy on Ubuntu 24.04 (Noble) will **not** work: `gazebo11` has no apt candidate there.
- `gazebo_ros`, `gazebo_plugins`
- `robot_state_publisher`, `joint_state_publisher`
- `xacro`
- `cartographer_ros` (for mapping)
- `nav2_bringup` (for navigation)
- `rviz2`

Install any missing packages with `rosdep`:

```bash
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/your_ws
colcon build --packages-select qcar_gazebo
source install/setup.bash
```

## Package layout

| Path | Purpose |
|---|---|
| `urdf/qcar_model.xacro` | Robot description: chassis, Ackermann hubs/wheels, lidar, cameras, and the `gazebo_ros_ackermann_drive` plugin block (drive kinematics, PID gains, odometry) |
| `launch/qcar_gazebo.launch.py` | Core sim bring-up: Gazebo, robot spawn, `robot_state_publisher`, `joint_state_publisher`, RViz |
| `launch/qcar_visualize.launch.py` | Hardware equivalent of the above (no Gazebo) - `robot_state_publisher`, `joint_state_publisher`, RViz only |
| `launch/qcar_slam.launch.py` | Mapping: Cartographer SLAM (sim or hardware) |
| `launch/qcar_nav2.launch.py` | Navigation: Nav2 stack against a saved map (sim or hardware) |
| `config/cartographer/qcar_2d.lua` | Cartographer SLAM parameters |
| `config/nav2/nav2_params.yaml` | Nav2 stack parameters (AMCL, costmaps, planner, controller) - every non-default value has an inline comment explaining why |
| `config/nav2/behavior_trees/*.xml` | Custom copies of nav2's stock BT trees - replan only on an invalid/updated path (not unconditionally every second), no `<Spin>` recovery (this Ackermann car can't rotate in place), and a per-replan path-smoothing step |
| `src/critics/early_commit_critic.cpp` | Custom MPPI critic plugin (`EarlyCommitCritic`) - see "Custom MPPI plugins" below |
| `src/smoothers/cusp_straightener_smoother.cpp` | Custom path-smoother plugin (`CuspStraightenerSmoother`) - see "Custom MPPI plugins" below |
| `maps/qcar_map_sim.yaml` / `.pgm` | Saved occupancy grid for the Gazebo world |
| `maps/qcar_map_hardware.yaml` / `.pgm` | Saved occupancy grid for the real QCar's test environment |
| `scripts/qcar_teleop_twist.py` | Keyboard teleop publishing `/cmd_vel` (works against sim or real hardware) |
| `scripts/qcar_relay_node.py` | Dev-PC side of the real-hardware bridge - see "Real hardware" below |
| `worlds/myworld.world` | The Gazebo world used by `qcar_gazebo.launch.py` |

## Quick start: simulation

```bash
ros2 launch qcar_gazebo qcar_nav2.launch.py launch_sim:=true
```

Brings up Gazebo, spawns the robot, and starts the full Nav2 stack against `qcar_map_sim.yaml`.
Set the robot's starting pose with RViz's **2D Pose Estimate** if it doesn't match AMCL's default
(`x=0, y=0, yaw=0`), then send a goal with **2D Nav Goal**.

To bring up just the simulation without Nav2 (e.g. for manual driving or building a new map):

```bash
ros2 launch qcar_gazebo qcar_gazebo.launch.py
```

You can drive manually at any time with `ros2 run qcar_gazebo qcar_teleop_twist.py` (publishes
to `/cmd_vel`, the same topic Nav2's controller uses).

### How the drive plugin is wired into the rest of the stack

The `gazebo_ros_ackermann_drive` plugin (declared directly in the `<gazebo>` block of
`urdf/qcar_model.xacro`) subscribes to `/cmd_vel` and publishes `/odom` and the `odom -> base` TF
transform directly - no topic relays or `ros2_control` controller manager needed. It also
broadcasts wheel/steering-hub TF (`publish_wheel_tf: true`) so RViz can visualize the wheels
turning, and wheelbase/track/wheel radius are auto-computed from the URDF's own joint poses and
collision geometry at spawn time rather than stated anywhere in a config file.

## Mapping (SLAM)

1. Launch SLAM:

   ```bash
   ros2 launch qcar_gazebo qcar_slam.launch.py launch_sim:=true   # or launch_sim:=false for hardware
   ```

   Starts the robot (sim or hardware) plus two Cartographer nodes (`cartographer_node`
   publishing `map -> odom -> base`, `cartographer_occupancy_grid_node` publishing `/map`).

2. Drive the robot around the environment to build up the map, e.g. with
   `ros2 run qcar_gazebo qcar_teleop_twist.py` (`w`/`s` forward/backward, `a`/`d` steer,
   `z` centre steering, `x` stop and centre, `q`/`e` speed up/down).

3. Watch progress in RViz (`Fixed Frame: map`; the laser scan and map display are already
   configured in `rviz/qcar.rviz`).

4. Once the map looks complete, save it:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/your_ws/src/qcar_gazebo/maps/qcar_map_sim
   ```

   (use `qcar_map_hardware` instead of `qcar_map_sim` for a real-hardware map). This is what
   `qcar_nav2.launch.py` loads by default, picked automatically based on `launch_sim`.

## Navigation (Nav2 + AMCL)

```bash
ros2 launch qcar_gazebo qcar_nav2.launch.py launch_sim:=true    # simulation
ros2 launch qcar_gazebo qcar_nav2.launch.py launch_sim:=false   # real QCar (default) - see "Real hardware" below
```

Set the robot's starting pose with **2D Pose Estimate** in RViz if it doesn't match AMCL's
default seed pose, let AMCL converge for a few seconds (driving a little helps it disambiguate),
then send a goal with **2D Nav Goal** or the `navigate_to_pose` action. The robot plans via
`nav2_smac_planner::SmacPlannerHybrid` (Reeds-Shepp motion model, can include a reverse/K-turn
segment) and drives there with `nav2_mppi_controller::MPPIController`.

`launch_sim` defaults to `false` - a bare `qcar_nav2.launch.py` targets the real hardware, not
Gazebo, matching how this package is normally used. Pass `launch_sim:=true` explicitly for sim.

`map_override:=<path/to/map.yaml>` overrides the automatic sim/hardware map selection, e.g. to
test the simulation against a hardware-captured map.

## Real hardware

This package drives the physical QCar too - it's the *default* mode, not a special case. The
physical QCar's onboard compute runs a different, incompatible ROS2 distribution from the dev
PC, and native ROS2 pub/sub between the two does not interoperate (raw network traffic passes
fine, but ROS2's own discovery protocol never completes). So instead of running this package's
stack on the QCar, a plain TCP/JSON bridge connects the two:

```
qcar_teleop_twist.py / Nav2's controller  (dev PC, ROS2, this package)
        |  /cmd_vel
        v
qcar_relay_node.py  (dev PC, ROS2 - the only real-hardware script that speaks ROS2)
        |  newline-delimited JSON over TCP (ports 5555 motor, 5556 lidar)
        v
qcar_bridge.py / qcar_lidar_node.py  (QCar onboard, plain Python, no ROS2 at all)
        |  Quanser HAL
        v
real motor / steering / encoder / lidar
```

Bring-up (see the [`qcar_hardware`](../qcar_hardware) package's README for the onboard side in
full, and its `commands.md` for a copy-pasteable command reference):

```bash
# on the QCar (two terminals - see qcar_hardware/README.md for exact paths):
sudo -E env PYTHONPATH=/home/nvidia/Documents/python python3 qcar_bridge.py
PYTHONPATH=/home/nvidia/Documents/python python3 qcar_lidar_node.py

# on the dev PC:
ros2 run qcar_gazebo qcar_relay_node.py --ros-args -p qcar_ip:=<qcar-ip> -p qcar_clock_offset:=<measured-offset>
ros2 launch qcar_gazebo qcar_nav2.launch.py
```

`qcar_clock_offset` corrects for the QCar's and dev PC's independent, unsynchronized clocks -
measure it fresh each session (procedure in `qcar_relay_node.py`'s module docstring), since it
drifts if either machine reboots. The QCar's IP is typically DHCP-assigned and can change per
session too.

**Throttle/steering calibration** lives in `qcar_bridge.py` on the QCar side (`THROTTLE_GAIN`,
`STEERING_TRIM`, etc.) - see the `qcar_hardware` README's "Recalibrating" section. These drift
with battery state and floor condition and may need periodic re-checking.

## Custom MPPI plugins

Two custom plugins, built into this package, extend stock Nav2 to handle this vehicle's
Ackermann kinematics and its K-turn/reversal maneuvers reliably:

- **`EarlyCommitCritic`** (`src/critics/early_commit_critic.cpp`, exported via `qcar_critics.xml`)
  - a custom MPPI critic. For a goal whose planned path curves immediately from the robot's
  current position, MPPI can fail to commit to the turn, since nav2's own `PathAngleCritic` gives
  little heading guidance for moderate initial mismatches. This critic scores only the first few
  control steps of each sampled trajectory against the near-term path bearing, unconditionally,
  so MPPI always starts turning toward the path immediately. See the header comment for the full
  gating rationale.

  Building this plugin requires `xtensor`/`xsimd` compile definitions to match
  `nav2_mppi_controller`'s own build exactly (see the comment block at the top of
  `CMakeLists.txt`) - a mismatch here is a silent ABI mismatch that crashes `nav2_container` on
  the first control cycle that touches vectorized critic math, not a compile error.

- **`CuspStraightenerSmoother`** (`src/smoothers/cusp_straightener_smoother.cpp`, exported via
  `qcar_smoothers.xml`) - a custom `nav2_core::Smoother` plugin. A front-steered vehicle is far
  less self-correcting when reversing than driving forward, and curving immediately from a
  standing start in reverse is the hardest sub-case of a K-turn. This plugin re-shapes the
  planned path at each reverse-entering cusp so the vehicle gets a short straight run to start
  reversing on before curving, gradually blending back into the original curve. See the header
  comment for the full geometry.

## Launch arguments

`qcar_nav2.launch.py` and `qcar_slam.launch.py` both accept:

| Argument | Default | Description |
|---|---|---|
| `launch_sim` | `false` | `true`: bring up Gazebo + the simulated robot. `false`: drive the real QCar (requires the onboard bridge and `qcar_relay_node.py` already running - see "Real hardware" above). |

`qcar_nav2.launch.py` additionally accepts:

| Argument | Default | Description |
|---|---|---|
| `map_override` | `""` (unused) | Optional map yaml path to use instead of the automatic sim/hardware pick. |

Example:

```bash
ros2 launch qcar_gazebo qcar_slam.launch.py launch_sim:=true
```

## Key robot parameters

These are derived from the STL meshes and joint origins in `urdf/qcar_model.xacro`, and (except
where noted) auto-computed by the drive plugin from that geometry at spawn time rather than
stated in any config file:

| Parameter | Value | Source |
|---|---|---|
| Wheel radius | 0.033 m (mesh), 0.036 m (collision cylinder) | `models/qcar/QCarWheel.stl` bounding box; collision geometry is padded +3mm for reliable ground contact |
| Wheelbase | 0.25725 m | Sum of front/rear hub joint x-offsets (0.12960 + 0.12765) |
| Front wheel track | 0.1118 m | `base_hubfl_joint`/`base_hubfr_joint` y-offsets (2 x 0.05590) |
| Rear wheel track | 0.1122 m | `base_wheelrl_joint`/`base_wheelrr_joint` y-offsets (2 x 0.05610) |
| Max steering angle | ±0.5236 rad (30°) | `base_hubfl_joint` / `base_hubfr_joint` limits, and the plugin's `max_steer` |
| Min turning radius | ~0.445 m | Derived: wheelbase / tan(max steering angle) |
| Lidar range | 0.15 - 12.0 m | Lidar sensor plugin, matches `nav2_params.yaml` |

Nav2 speed limits (`config/nav2/nav2_params.yaml`): max linear velocity 0.3 m/s forward, 0.2 m/s
reverse (`FollowPath.vx_max`/`vx_min`), max angular velocity 1.0 rad/s.

## Topics and frames reference

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Driving input; subscribed to directly by the drive plugin (sim) or `qcar_relay_node.py` (hardware) |
| `/odom` | `nav_msgs/Odometry` | Wheel-based odometry (real dead reckoning, not ground truth) |
| `/scan` | `sensor_msgs/LaserScan` | From the simulated or real lidar |
| `/map` | `nav_msgs/OccupancyGrid` | From Cartographer (mapping) or the map server (navigation) |

TF tree: `map -> odom -> base -> {lidar, hubfl, hubfr, wheelfl, wheelfr, wheelrl, wheelrr,
camera_*}`. During SLAM, Cartographer owns `map -> odom`; during navigation, AMCL does. `odom ->
base` comes from the drive plugin in sim, or from `qcar_relay_node.py`'s dead-reckoning on
hardware.

## Troubleshooting

- **Robot doesn't move (sim)**: check the Gazebo terminal for
  `[gazebo_ros_ackermann_drive]: Subscribed to [/cmd_vel]` at startup - if missing, the plugin
  failed to load (check for URDF parse errors just above it).
- **Robot moves but barely turns / doesn't reach Nav2 goals accurately (sim)**: check the
  steering PID gain (`left/right_steering_pid_gain` in the `<gazebo>` plugin block of
  `urdf/qcar_model.xacro`) - too weak relative to wheel/ground friction silently produces a much
  larger real turning radius than commanded.
- **No `/scan` data (sim)**: check `gazebo_ros_ray_sensor` plugin output in the Gazebo terminal
  for plugin load errors.
- **AMCL doesn't localize / lidar points seem to jump or drift while driving**: first verify the
  initial pose (2D Pose Estimate) matches the robot's actual position in the map, and give AMCL a
  few seconds and a bit of driving to converge (check `/amcl_pose`'s covariance shrinking). Some
  correction on every scan match is expected (real odometry drift), not a bug.
- **Weird/inconsistent behavior across repeated test launches (duplicate TF, jittery joints)**:
  make sure no processes from a previous run are still alive before relaunching -
  `ps aux | grep -E "gazebo|gzserver|gzclient|rviz2|robot_state_publisher|joint_state_publisher"`
  should show nothing before a fresh `ros2 launch`.
- **Hardware-specific issues** (bridge connection, throttle/steering calibration, stall/overcurrent
  behavior): see the `qcar_hardware` package's README.
