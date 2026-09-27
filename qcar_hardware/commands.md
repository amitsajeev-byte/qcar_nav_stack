# QCar hardware terminal commands

Copy-pasteable reference for every terminal command used to bring up, test, and shut down the
real-hardware QCar stack. See this package's `README.md` for the full explanation of what each
piece does and where the files belong on the QCar.

Replace `<qcar-ip>` with the QCar's actual IP throughout - it's typically DHCP-assigned and can
change per session (`ping <qcar-ip>` to confirm, or check the QCar's own network settings if the
connection fails). Replace `<offset>` with a freshly-measured `qcar_clock_offset` (see below) -
also re-measure this any time either machine reboots.

## One-time build + deploy (after any local code change)

```bash
# on the dev PC, from your ROS2 workspace root:
colcon build --symlink-install --packages-select qcar_gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash

# push the onboard scripts to the QCar - only needed if a qcar_hardware/*.py file changed
scp qcar_hardware/*.py nvidia@<qcar-ip>:~/ros2_amit/src/onboard_bridge/
```

## Terminal 1 (QCar motor bridge - needs sudo for hardware access)

```bash
ssh nvidia@<qcar-ip>
sudo -E env PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_bridge.py
```

(You'll be prompted for the QCar's own sudo password - not recorded here.)

## Terminal 2 (QCar LiDAR node - no sudo needed)

```bash
ssh nvidia@<qcar-ip>
PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_lidar_node.py
```

## Measuring qcar_clock_offset (dev PC, repeat each session)

Use a single persistent SSH connection for every sample - a fresh connection per sample folds
its own handshake time into the measurement asymmetrically and can inflate the apparent offset
by an order of magnitude:

```bash
ssh -o ControlMaster=yes -o ControlPath=/tmp/qcar_ssh -o ControlPersist=60s -fN nvidia@<qcar-ip>
for i in 1 2 3 4 5; do
  t1=$(date +%s.%N); remote=$(ssh -o ControlPath=/tmp/qcar_ssh nvidia@<qcar-ip> 'date +%s.%N'); t2=$(date +%s.%N)
  python3 -c "print(($remote) - (($t1 + $t2) / 2))"
done
ssh -o ControlPath=/tmp/qcar_ssh -O exit nvidia@<qcar-ip>
```

Average the 5 printed values (should cluster tightly within a few tens of milliseconds - a wide
spread means redo it) and use in place of `<offset>` below.

## Terminal 3 (dev PC relay node)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qcar_gazebo qcar_relay_node.py --ros-args -p qcar_ip:=<qcar-ip> -p qcar_clock_offset:=<offset>
```

## Terminal 4 (dev PC) - SLAM (real hardware path, no Gazebo)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar_gazebo qcar_slam.launch.py launch_sim:=false
```

## Terminal 4 alt (dev PC) - Nav2 (real hardware path, no Gazebo)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar_gazebo qcar_nav2.launch.py
```

(`launch_sim` defaults to `false`, so this targets real hardware without needing the flag.)

## Terminal 5 (dev PC) - teleop

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qcar_gazebo qcar_teleop_twist.py
```

## Save a map (dev PC, while qcar_slam.launch.py is up)

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/your_ws/src/qcar_gazebo/maps/qcar_map_hardware
```

## AMCL global localization (skip needing a precisely marked start pose)

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

Drive a short distance afterward before trusting localization/sending a goal.

## Recalibrating throttle/steering (on the QCar, `qcar_bridge.py` stopped first)

```bash
ssh nvidia@<qcar-ip>
cd ~/ros2_amit/src/onboard_bridge/
python3 calibrate_steering.py <steering_command_rad>       # interactive, wheels off the ground
python3 calibrate_throttle.py fwd <confirmed_clear_distance_m>
python3 calibrate_throttle.py rev <confirmed_clear_distance_m>
```

See this package's README for how to read the results back into `qcar_bridge.py`.

## Useful live-diagnostic commands (dev PC)

```bash
ros2 node list
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /odom --field pose.pose.position --once
ros2 topic echo /amcl_pose --field pose.pose.pose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo base lidar
```

## Shutdown, in order

```bash
# dev PC: Ctrl+C teleop / nav2 launch / relay node, in that order

# QCar side:
ssh nvidia@<qcar-ip> 'pkill -f qcar_lidar_node.py'
ssh nvidia@<qcar-ip> 'sudo pkill -f qcar_bridge.py'   # will prompt for the QCar's sudo password
ssh nvidia@<qcar-ip> 'sudo ss -tlnp | grep -E "5555|5556"'   # should print nothing
```
