# QCar Navigation Stack

A ROS 2 (Humble) / Gazebo Classic / Nav2 navigation stack for the Quanser QCar, an Ackermann
(car-like) steered ground robot. Includes SLAM mapping, MPPI-based navigation with two custom
plugins for reliable K-turn/reversal handling, and a TCP bridge that lets the same navigation
stack drive either the Gazebo simulation or the real physical QCar without any code changes.

## Repository layout

| Package | Runs where | Purpose |
|---|---|---|
| [`qcar_gazebo`](qcar_gazebo) | Dev PC, ROS 2 | The full navigation stack: robot description, Gazebo simulation, Cartographer SLAM, Nav2 (planner, controller, custom critic/smoother plugins), and the dev-PC-side relay node for real hardware |
| [`qcar_hardware`](qcar_hardware) | The QCar's own onboard computer, plain Python (no ROS2) | The onboard bridge scripts that expose the real QCar's motor/steering/lidar hardware over a TCP socket, plus calibration tools |

Both are needed for a real-hardware run; only `qcar_gazebo` is needed for simulation.

## Why two packages instead of one

The QCar's onboard computer runs a different, incompatible ROS2 distribution from a typical dev
PC, and native ROS2 communication between the two does not work (confirmed on hardware: the raw
network path is fine, but ROS2's own discovery protocol never completes). So the onboard side
can't be a normal ROS2 node at all - `qcar_hardware`'s two scripts talk to the real motor/lidar
hardware directly through Quanser's own HAL and expose it over a plain TCP socket instead.
`qcar_gazebo`'s `qcar_relay_node.py` is the only piece that speaks both that TCP protocol and
ROS2, bridging the two worlds. See `qcar_hardware/README.md` for the full picture.

## Quick start

**Simulation only** (no real hardware needed):

```bash
cd ~/your_ws/src
git clone https://github.com/amitsajeev-byte/qcar_nav_stack.git
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select qcar_gazebo
source install/setup.bash
ros2 launch qcar_gazebo qcar_nav2.launch.py launch_sim:=true
```

Then send a goal with RViz's **2D Nav Goal** tool. See `qcar_gazebo/README.md` for the full
walkthrough (mapping, launch arguments, troubleshooting).

**Real hardware**: see `qcar_hardware/README.md` for deploying the onboard bridge to the QCar,
then `qcar_gazebo/README.md`'s "Real hardware" section for the dev-PC side. `launch_sim` defaults
to `false`, so a bare `qcar_nav2.launch.py` targets real hardware by default.

## What makes this stack's Nav2 configuration non-default

Two things about an Ackermann vehicle break most Nav2 tutorials' default assumptions, and this
stack's configuration is built around both:

1. **It can't rotate in place.** Any reorientation-only goal, and any recovery behavior that
   would normally spin, needs an actual arcing maneuver instead. `<Spin>` is removed from the
   active behavior tree's recovery actions for exactly this reason.
2. **Reversing is genuinely harder than driving forward for a front-steered vehicle** - the
   steered wheels are the self-correcting leading axle going forward, but the
   jackknife-prone trailing axle in reverse. `SmacPlannerHybrid` (a heading-aware, Reeds-Shepp
   capable global planner) is used instead of a simpler planner precisely so it can plan real
   K-turn segments when a goal's orientation demands one, and a custom path-smoother plugin
   (`CuspStraightenerSmoother`) reshapes the path at each such cusp so the controller gets a
   short straight run to start reversing on rather than curving immediately from a standstill.

See `qcar_gazebo/README.md`'s "Custom MPPI plugins" section and the inline comments in
`qcar_gazebo/config/nav2/nav2_params.yaml` for the full detail behind every non-default parameter.

## License

MIT - see [LICENSE](LICENSE).
