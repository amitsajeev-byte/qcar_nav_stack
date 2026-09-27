# qcar_hardware

The onboard side of the real QCar bridge - the scripts that run **directly on the QCar's own
onboard computer**, not in ROS2 at all. This is not a ROS2 package (no `package.xml`, nothing to
`colcon build`); it's deployed by copying these files onto the QCar and running them as plain
Python processes.

For the dev-PC side (the actual ROS2/Nav2 stack, and the relay node that talks to these scripts
over the network), see the sibling [`qcar_gazebo`](../qcar_gazebo) package.

## Why this exists as a separate, non-ROS2 layer

The QCar's onboard computer runs a different, incompatible ROS2 distribution from a typical dev
PC. Native ROS2 pub/sub between the two does not interoperate: raw network traffic passes fine in
both directions, but ROS2's own discovery protocol (Fast-RTPS/DDS) never completes between the
two distributions - confirmed directly on hardware, not a firewall/network issue. So instead of
trying to run any ROS2 node on the QCar itself, these two scripts talk to the real hardware
directly through Quanser's own HAL (`pal.products.qcar`) and expose it over a plain TCP socket,
in a simple newline-delimited JSON protocol. `qcar_relay_node.py` (in `qcar_gazebo`, running on
the dev PC) is the only thing that speaks both this TCP protocol and ROS2 - it's the actual
bridge between the two worlds.

```
qcar_teleop_twist.py / Nav2's controller   (dev PC, ROS2, qcar_gazebo package)
        |  /cmd_vel
        v
qcar_relay_node.py   (dev PC, ROS2 - the only real-hardware script that speaks ROS2)
        |  newline-delimited JSON over TCP (port 5555 motor+odom, port 5556 lidar)
        v
qcar_bridge.py / qcar_lidar_node.py   (QCar onboard, plain Python, no ROS2 at all - this package)
        |  Quanser HAL (pal.products.qcar)
        v
real motor / steering servo / encoder / lidar
```

## Files

| File | Runs where | Purpose |
|---|---|---|
| `qcar_bridge.py` | QCar, needs `sudo` | Drive bridge - listens on TCP port 5555, converts `/cmd_vel`-equivalent commands into HAL throttle/steering calls, streams odometry back |
| `qcar_lidar_node.py` | QCar, no `sudo` needed | LiDAR bridge - listens on TCP port 5556, streams scan data. A separate process from `qcar_bridge.py` on purpose, so a LiDAR issue can't take down drive control and vice versa |
| `calibrate_steering.py` | QCar | Interactive one-off tool: holds a fixed steering command so you can visually check the real wheel angle against straight |
| `calibrate_throttle.py` | QCar | Sweeps fixed PWM duty values and fits `duty = DEADBAND + GAIN * speed` from measured real speed |
| `commands.md` | - | Copy-pasteable terminal commands for every step below |

## Where these files live on the QCar

This project's convention (confirm/adjust for your own unit): `~/ros2_amit/src/onboard_bridge/`
on the QCar, i.e. `/home/nvidia/ros2_amit/src/onboard_bridge/qcar_bridge.py` etc. Deploy with:

```bash
scp qcar_hardware/*.py nvidia@<qcar-ip>:~/ros2_amit/src/onboard_bridge/
```

`PYTHONPATH=/home/nvidia/Documents/python` must be set when running any of these scripts - that's
where the QCar's `pal.products.qcar` HAL module actually lives on this unit.

## Running

```bash
# Terminal 1 - motor bridge, needs sudo for hardware access
ssh nvidia@<qcar-ip>
sudo -E env PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_bridge.py

# Terminal 2 - lidar bridge, no sudo needed
ssh nvidia@<qcar-ip>
PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_lidar_node.py
```

Both print `listening on port <N> (waiting for dev PC relay)` and then block until
`qcar_relay_node.py` connects from the dev PC (see `qcar_gazebo`'s README for that side). Both
handle `Ctrl+C` and `SIGTERM` cleanly (zero the motor / stop the LiDAR motor before exiting) - a
plain `kill` without `-9` is safe; avoid `kill -9`, which skips that cleanup entirely.

Stopping:

```bash
ssh nvidia@<qcar-ip> 'pkill -f qcar_lidar_node.py'
ssh nvidia@<qcar-ip> 'sudo pkill -f qcar_bridge.py'
```

## Safety behavior built into `qcar_bridge.py`

These are real safety backstops, not just tuning knobs - see the inline comments in the file
itself for the full reasoning:

- **Command timeout** (`CMD_TIMEOUT`, 0.5s): if no command arrives (dropped WiFi, crashed relay),
  the car stops.
- **Stall cutoff** (`STALL_THROTTLE_THRESHOLD`/`STALL_TIMEOUT`): if throttle has been commanded
  for 3 seconds straight with no corresponding encoder motion, the throttle is cut. Protects
  against the documented risk of holding the drive motor in a stalled position.
- **Overcurrent/battery warnings**: console warnings mirroring the QCar hardware manual's own
  FPGA overcurrent tiers and the power manual's low-battery thresholds. These are visibility only
  - they cannot override the FPGA's own protection, which trips independently; the QCar's own LCD
  is the authoritative source for whether that happened.
- **Steering/throttle rate limits**: both are rate-limited per control cycle so a sudden target
  change (e.g. an upstream controller oscillating) becomes a bounded ramp, not an instant
  full-range reversal - a real mechanical stress risk for the servo/motor otherwise.

## Recalibrating

Throttle and steering calibration constants live at the top of `qcar_bridge.py`
(`STEERING_GAIN`, `STEERING_TRIM`, `THROTTLE_DEADBAND`, `THROTTLE_GAIN`, `REVERSE_DEADBAND`,
`REVERSE_GAIN`) with a comment on each explaining what it corrects for. These are genuinely
session-dependent - battery voltage and floor/tire friction both drift the real fit - so
periodic recalibration is expected, not a sign something is broken.

**Steering** (run with `qcar_bridge.py` stopped - both need exclusive HAL access):

```bash
python3 calibrate_steering.py <steering_command_rad>
```

Try a few candidate values, watching the real wheel angle (wheels off the ground or the car
otherwise safely restrained), until it sits visually straight. Record that value as
`STEERING_TRIM`. `STEERING_GAIN` (the ratio between commanded and real wheel deflection) needs a
small multi-point sweep to refit properly if it drifts - hold a few different nonzero commands
and measure the real deflection at each, then fit `implied_real_angle = GAIN*cmd + TRIM`.

**Throttle** (run with `qcar_bridge.py` stopped, and with confirmed clear space in the direction
being tested - the script enforces a hard distance-budget cutoff from whatever you tell it, but
that's a backstop, not a substitute for actually checking):

```bash
python3 calibrate_throttle.py fwd <confirmed_clear_distance_m>
python3 calibrate_throttle.py rev <confirmed_clear_distance_m>
```

Prints a fitted `duty = DEADBAND + GAIN * speed` for each direction - update
`THROTTLE_DEADBAND`/`THROTTLE_GAIN` (forward) or `REVERSE_DEADBAND`/`REVERSE_GAIN` (reverse) in
`qcar_bridge.py` if the fit has drifted meaningfully (more than roughly 10% off) from the current
hardcoded values, then redeploy (`scp`) and restart the bridge.

Forward and reverse do not reliably share one gain pair - recalibrate both directions in the same
session, on the same battery, rather than assuming one carries over to the other.

**A real physical limit, not a calibration gap**: below roughly 0.25-0.3 m/s commanded, this
vehicle sits at a genuine static-friction (stiction) deadband and can fail to move at all,
regardless of how well-calibrated the gains are. Don't chase a "fix" for unreliable behavior
below that speed - it's a hardware limit of this chassis/floor combination.

## Known risks / things to double-check on a different unit

- **Steering mechanical center**: this chassis has no adjustable steering linkage, so
  `STEERING_TRIM` is a pure software correction for wherever the wheels happen to sit at a
  commanded angle of 0. If the steering linkage is ever physically adjusted or replaced,
  recalibrate from scratch.
- **LiDAR angle convention**: `qcar_lidar_node.py`'s direction flip + 90-degree rotational offset
  (see the comment in `read_scan()`) was verified against a real obstacle placement on this
  specific unit's LiDAR mounting. If you're deploying to a different QCar or the LiDAR is
  remounted, re-verify this before trusting `/scan` - place a known object at the vehicle's
  physical right and confirm it appears at -90 degrees in RViz, not the back.
- **`sudo` requirement asymmetry**: `qcar_bridge.py` needs `sudo` for motor/HAL access;
  `qcar_lidar_node.py` has run fine without it on this unit, even though the QCar's own
  troubleshooting documentation suggests LIDAR applications need `sudo` too. If
  `qcar_lidar_node.py` ever fails to open the LIDAR device with a permissions-looking error, try
  `sudo` before assuming a wiring/connector fault.
