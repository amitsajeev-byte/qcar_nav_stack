#!/usr/bin/env python3
'''qcar_bridge.py

Minimal QCar drive bridge. Runs directly on the QCar's onboard computer:

    python3 qcar_bridge.py

No ROS2 involved on this side at all - native ROS2 pub/sub between the
QCar's onboard ROS2 distribution and the dev PC's does not interoperate
(the two run different, incompatible ROS2 distributions, and Fast-RTPS
discovery between them never completes even though the raw network path
works fine). Instead, this listens on a plain TCP socket for
newline-delimited JSON drive commands from qcar_relay_node.py on the dev
PC (the only thing that still speaks ROS2 - it bridges this socket to
/cmd_vel on the ROS2 side).

Wire format, one JSON object per line, one direction each way on the same
connection (TCP is full-duplex - no separate port needed):
    dev PC -> QCar: {"linear_x": <float m/s equivalent>, "angular_z": <float rad/s>}
    QCar -> dev PC: {"t": <QCar time.time() at capture>, "x": <m>, "y": <m>,
                      "yaw": <rad>, "v": <m/s>, "yaw_rate": <rad/s>}

The odometry direction exists because AMCL looks up a live odom->base TF
at every scan callback to track motion between localization corrections.
This is open-loop dead reckoning: there's no separate steering-angle
feedback sensor in this HAL, so the yaw-rate integration below uses the
last-*commanded* steering angle, not a measured one - accuracy depends on
the vehicle's steering mechanical center actually matching steering=0.0
(see STEERING_TRIM below).

The "t" field is this QCar's own time.time() at the moment of capture, not
a receipt-time stamp: timestamping at receipt time on the dev PC side
(independent, uncorrelated latency vs. the LiDAR connection) causes AMCL
to pair scans with poses from a slightly different real-world instant,
producing a rigid offset between the live scan and the map. See
qcar_relay_node.py's docstring (in the qcar_gazebo package) for how this
timestamp gets converted to a dev-PC-clock-equivalent stamp.

See the qcar_hardware README for the full architecture and deployment
instructions, and the QCar's own Quanser HAL/PAL documentation for the API
this wraps.
'''
import json
import math
import signal
import socket
import time

from pal.products.qcar import QCar, IS_PHYSICAL_QCAR


# SIGTERM (pkill/kill's default signal) is not auto-converted to a catchable KeyboardInterrupt
# the way SIGINT (Ctrl+C) is - without this, a plain `pkill -f qcar_bridge.py` would skip the
# cleanup in main()'s `finally` block entirely (car.write(0,0,...) + car.terminate()),
# potentially leaving the motor drive in an uncertain state.
def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt()


signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

# Wheelbase (m): matches qcar_gazebo's urdf/qcar_model.xacro hub joint origins and
# qcar_teleop_twist.py.
WHEELBASE = 0.25725

# Steering *command* range accepted by the HAL is -0.5 to 0.5 rad - distinct from the vehicle's
# physical max steering angle (0.5236 rad / 30 deg).
MAX_STEER_CMD = 0.5

# Compensates for two independent, real effects on this specific chassis: (1) the steering
# mechanical center being offset from steering=0.0 (wheels sit visibly off-straight when
# commanded to 0), and (2) a real steering GAIN error - the physical wheel turns more than the
# commanded angle for any nonzero command, not just a zero-point offset. No adjustable physical
# linkage is available and no factory steering calibration curve is documented, so both are
# corrected in software, calibrated empirically (see calibrate_steering.py and the "Recalibrating"
# section of the qcar_hardware README). Implied real wheel angle fits as
# `STEERING_GAIN * cmd + (real angle at cmd=0)`; STEERING_TRIM is the zero-crossing of that fit.
# Recalibrate if the vehicle stops tracking straight, or if the chassis/steering linkage changes.
STEERING_GAIN = 1.39
STEERING_TRIM = -0.087

# PWM duty-cycle safety limits (see the QCar hardware manual): saturate to +/-30% magnitude,
# rate-limited to 100% duty-cycle change per second, to avoid a battery-brownout-triggered
# shutdown.
THROTTLE_LIMIT = 0.3
THROTTLE_RATE_LIMIT = 1.0  # duty-cycle fraction per second

# Steering had no rate limit at all before this - target_steering went straight from
# compute_steering() to car.write() every control cycle with nothing damping a full-range
# reversal. Repeated full-swing reversals under load are a real mechanical stress risk
# (encoder/servo wear) independent of whatever is driving the oscillation upstream. This is a
# hardware-boundary backstop, not a fix for an upstream controller-side cause - if normal
# path-following starts feeling sluggish/late on real turns, raise this rather than removing it.
# Bench-test any change here with the wheels off the ground before trusting it on a real drive.
STEERING_RATE_LIMIT = 2.0  # rad per second

# cmd_vel's linear.x is m/s; the HAL's throttle is a PWM duty-cycle fraction (unitless) - there's
# no documented conversion between the two. There is also a real static-friction (stiction) floor
# below which no commanded speed produces motion at all, and once past it the real speed is very
# sensitive to small duty changes (a stick-slip signature: static friction exceeds kinetic
# friction, so the instant the car breaks free, the same duty suddenly has much more net force to
# accelerate with) - below roughly 0.25-0.3 m/s commanded, treat the vehicle's response as
# unreliable, not a tuning gap to chase further.
#
# duty = THROTTLE_DEADBAND + THROTTLE_GAIN * |speed|, fitted and validated for the 0.3-0.5 m/s
# range by driving fixed durations at several commanded speeds and comparing real encoder-measured
# cruise speed against the command. This is genuinely session-dependent (battery voltage and floor
# friction both drift the fit) - re-run calibrate_throttle.py periodically and update these two
# constants if real behavior no longer matches commanded speed within about 10%. FORWARD ONLY -
# see REVERSE_DEADBAND/REVERSE_GAIN below for why reverse needs its own fit.
THROTTLE_DEADBAND = 0.04
THROTTLE_GAIN = 0.077

# Reverse does not reliably share forward's DEADBAND/GAIN - real testing has shown reverse
# noticeably more sensitive to battery state and floor condition than forward, sometimes needing
# a distinctly different gain and sometimes matching forward exactly once the battery is fresh.
# Re-fit independently with calibrate_throttle.py (both directions, same session, same battery)
# whenever recalibrating - don't assume the same constants apply to both directions without
# checking.
REVERSE_DEADBAND = 0.055
REVERSE_GAIN = 0.070

# OVERCURRENT_TIERS mirrors the QCar hardware manual's own FPGA protection tiers (motor forced to
# Neutral/coast if current sustains 5A for 8s, 10A for 2s, or 15A for 0.5s) - this only prints an
# early warning on this process's own console; it cannot see or override the FPGA's own
# protection, which trips independently. The QCar's own LCD is the authoritative source for
# whether that actually happened ("Overcurrent" message) - recovering from it requires restarting
# this whole process (closing/reopening the HIL device), not just re-sending commands.
OVERCURRENT_TIERS = [(5.0, 8.0), (10.0, 2.0), (15.0, 0.5)]  # (amps, sustained seconds)
OVERCURRENT_WARN_FRACTION = 0.7  # warn at 70% of the FPGA's own sustained-time trip threshold

# Mirrors the QCar power manual's documented thresholds: 'LOW BAT' LCD warning below 10.5V,
# auto-shutdown below 10.0V. Independent of anything in this script - printed here purely for
# console visibility alongside the overcurrent warning above.
BATTERY_WARN_VOLTAGE = 10.5
BATTERY_SHUTDOWN_VOLTAGE = 10.0

# The QCar hardware manual documents that holding the motor in a stalled position for a
# prolonged period at applied voltages over 5V can cause permanent damage. CMD_TIMEOUT below only
# protects against a STALE command (nothing arriving) - it does nothing for a continuously
# refreshed nonzero command that's failing to actually move the vehicle (e.g. commanded duty
# sitting right at/below the real stiction floor). This is the software-side backstop for that
# gap: if throttle has been commanded above STALL_THROTTLE_THRESHOLD for STALL_TIMEOUT seconds
# straight with no corresponding encoder motion, cut the throttle. Deliberately conservative
# (encoder epsilon well above quadrature noise, generous timeout) - a safety backstop, not a
# substitute for fixing an underlying stiction/gain mismatch if one exists.
STALL_THROTTLE_THRESHOLD = 0.045  # duty fraction - above the deadband, i.e. "genuinely trying to drive"
STALL_ENCODER_EPS = 20  # encoder counts/cycle (~0.007 m/s at 50Hz) - treat as "not moving" below this
STALL_TIMEOUT = 3.0  # seconds of continuous stall before cutting throttle

# Active braking (a brief reverse-throttle pulse proportional to residual speed when a stop is
# commanded, to counter drivetrain coast) was tried and rejected - not just untuned. It's a
# proportional loop on velocity sign with no deadband and one-cycle-stale feedback: if the pulse
# overshoots past zero into real reverse motion, the next cycle reads a negative v and "corrects"
# with a forward push, which can itself overshoot - a genuine undamped hunting oscillation, not a
# bad gain value. Confirmed on hardware twice (the car visibly lurched forward/back repeatedly and
# had to be killed manually both times). Do not re-add this without a real deadband/hysteresis or
# a one-shot (non-continuous) brake pulse design. The car coasting for roughly 0.5-0.6s after a
# stop command is accepted as normal, safe behavior for now - budget for it in goal placement
# rather than trying to eliminate it reactively.

# If no command arrives for this long (or the socket disconnects), stop the car. Protects against
# a dropped WiFi link or a crashed dev-PC relay.
CMD_TIMEOUT = 0.5  # seconds

READ_RATE = 50.0  # Hz
LISTEN_PORT = 5555

# Encoder counts -> distance, derived from the drive motor's gear ratio and wheel radius (see the
# QCar hardware manual): distance(m) = encoderCounts * (1/2880) * 0.01977
METERS_PER_COUNT = 0.01977 / 2880.0


def compute_steering(linear_x, angular_z):
    # Returns the KINEMATIC (untrimmed) steering angle - the real physical wheel angle the
    # bicycle model wants, used for odometry and the LED turn-indicator logic. STEERING_TRIM must
    # NOT be folded in here: trim's whole purpose is to make the REAL wheels read ~0 despite a
    # nonzero servo command, so treating that same command as a real kinematic deflection would
    # corrupt dead-reckoning (a straight-line drive would integrate a fictitious curve). See
    # apply_trim() below for where STEERING_TRIM actually belongs - only at the car.write() call
    # site, never upstream of it.
    #
    # A car-like steering axle can't produce a meaningful angle at a standstill, so hold steering
    # at 0 rather than divide by ~0.
    if abs(linear_x) < 1e-2:
        steer = 0.0
    else:
        steer = math.atan(WHEELBASE * angular_z / linear_x)
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, steer))


def apply_trim(kinematic_steering):
    '''Converts a real/kinematic steering angle into the servo command that actually achieves it
    on this specific vehicle - only call this right before car.write(), never before odometry or
    anything else that cares about the real physical wheel angle. Inverts the measured
    implied_real_angle = STEERING_GAIN*cmd + (real angle at cmd=0) relationship by pre-dividing by
    the gain, so the real wheel ends up at kinematic_steering, not just at
    kinematic_steering + a flat offset.'''
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, kinematic_steering / STEERING_GAIN + STEERING_TRIM))


class Odometry:
    '''Bicycle-model dead reckoning from encoder deltas + last-commanded steering angle. Reset at
    the start of each new relay connection - odom is a purely local/relative frame by ROS
    convention, doesn't need to persist across reconnects.'''

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_encoder = None

    def update(self, encoder_counts, steering, dt):
        if self.last_encoder is None:
            self.last_encoder = encoder_counts
        delta_counts = encoder_counts - self.last_encoder
        self.last_encoder = encoder_counts

        distance = delta_counts * METERS_PER_COUNT
        v = distance / dt
        yaw_rate = v * math.tan(steering) / WHEELBASE

        self.yaw += yaw_rate * dt
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        return {'x': self.x, 'y': self.y, 'yaw': self.yaw, 'v': v, 'yaw_rate': yaw_rate}


def main():
    if not IS_PHYSICAL_QCAR:
        import qlabs_setup
        qlabs_setup.setup()

    # readMode=0 (immediate I/O), not 1 (task-based I/O): task-based I/O runs a background
    # acquisition task into a ring buffer from the moment the QCar object is constructed,
    # independent of when car.read() actually starts being called. Since car.read() isn't called
    # until a relay connects, the buffer can fill and start overwriting during that gap, leaving
    # every subsequent read permanently stuck behind real-time by a fixed lag. Immediate I/O reads
    # current hardware state directly with no task/buffer involved.
    car = QCar(readMode=0, frequency=READ_RATE)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', LISTEN_PORT))
    server.listen(1)
    print('qcar_bridge listening on port %d (waiting for dev PC relay)' % LISTEN_PORT)

    target_linear = 0.0
    target_steering = 0.0
    current_throttle = 0.0
    current_steering = 0.0
    last_cmd_time = time.time()
    last_loop_time = time.time()
    period = 1.0 / READ_RATE

    # State for the precaution checks documented above (OVERCURRENT_TIERS/STALL_*/BATTERY_*) -
    # reset per relay connection alongside odom, since a fresh connection is a natural "start
    # clean" point.
    last_stall_encoder = None
    stall_start_time = None
    overcurrent_tier_start = [None] * len(OVERCURRENT_TIERS)
    overcurrent_tier_warned = [False] * len(OVERCURRENT_TIERS)
    battery_warn_state = None  # None / 'warn' / 'shutdown' - avoid re-printing every cycle

    try:
        while True:
            conn, addr = server.accept()
            conn.settimeout(period)
            print('dev PC relay connected from', addr)
            buf = b''
            last_cmd_time = time.time()
            odom = Odometry()
            last_stall_encoder = None
            stall_start_time = None
            overcurrent_tier_start = [None] * len(OVERCURRENT_TIERS)
            overcurrent_tier_warned = [False] * len(OVERCURRENT_TIERS)
            battery_warn_state = None

            try:
                while True:
                    # Drain any newline-delimited JSON commands available without blocking the
                    # fixed-rate control loop below.
                    try:
                        chunk = conn.recv(4096)
                        if chunk == b'':
                            print('dev PC relay disconnected')
                            break
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            if not line.strip():
                                continue
                            try:
                                cmd = json.loads(line)
                                target_linear = float(cmd['linear_x'])
                                angular_z = float(cmd['angular_z'])
                                target_steering = compute_steering(target_linear, angular_z)
                                last_cmd_time = time.time()
                            except (ValueError, KeyError, TypeError):
                                print('bad command line, ignoring:', line)
                    except socket.timeout:
                        pass  # no new data this cycle - fall through to control loop
                    except OSError:
                        # Abrupt disconnect (e.g. the relay process killed rather than closed
                        # cleanly) raises an OSError here instead of the b'' empty-recv case
                        # above - handled the same way as a clean disconnect.
                        print('dev PC relay disconnected (recv failed)')
                        break

                    # Pace the actual hardware read/write to READ_RATE regardless of how fast
                    # commands arrive over the socket - a burst of queued messages shouldn't turn
                    # into a burst of car.read()/write() calls.
                    now = time.time()
                    dt = now - last_loop_time
                    if dt < period:
                        continue
                    last_loop_time = now

                    # Watchdog: stop driving if no command has arrived recently.
                    if now - last_cmd_time > CMD_TIMEOUT:
                        target_linear = 0.0
                        target_steering = 0.0

                    car.read()

                    # Stall backstop (see STALL_* comment above) - based on the PREVIOUS cycle's
                    # write, since car.read() reflects state up to now, before this cycle's
                    # car.write() happens further down.
                    now_encoder = float(car.motorEncoder[0])
                    if last_stall_encoder is None:
                        last_stall_encoder = now_encoder
                    moving = abs(now_encoder - last_stall_encoder) > STALL_ENCODER_EPS
                    last_stall_encoder = now_encoder

                    if abs(current_throttle) > STALL_THROTTLE_THRESHOLD and not moving:
                        if stall_start_time is None:
                            stall_start_time = now
                        elif now - stall_start_time > STALL_TIMEOUT:
                            # Timestamped so a STALL line can be cross-referenced against the dev
                            # PC's Nav2 launch log (which prints raw epoch seconds) - this is the
                            # QCar's own time.time(), same clock qcar_clock_offset corrects for,
                            # so subtract/add that offset when lining it up against dev PC logs.
                            print('[%s | epoch %.3f] STALL: throttle %.2f commanded for >%.1fs '
                                  'with no encoder motion - cutting throttle (see the QCar '
                                  'hardware manual\'s stalled-motor caution)' % (
                                      time.strftime('%H:%M:%S', time.localtime(now)) +
                                      '.%03d' % int((now % 1) * 1000),
                                      now, current_throttle, STALL_TIMEOUT))
                            target_linear = 0.0
                            current_throttle = 0.0
                            stall_start_time = None
                    else:
                        stall_start_time = None

                    # Overcurrent early warning - mirrors the FPGA's own tiers, see
                    # OVERCURRENT_TIERS comment above. Console-only, cannot override the FPGA.
                    current_amps = abs(float(car.motorCurrent))
                    for i, (amps_thresh, seconds_thresh) in enumerate(OVERCURRENT_TIERS):
                        if current_amps >= amps_thresh:
                            if overcurrent_tier_start[i] is None:
                                overcurrent_tier_start[i] = now
                                overcurrent_tier_warned[i] = False
                            elif (not overcurrent_tier_warned[i] and now - overcurrent_tier_start[i]
                                    > seconds_thresh * OVERCURRENT_WARN_FRACTION):
                                print('WARNING: motor current %.1fA approaching the %.0fA/%.1fs '
                                      'overcurrent tier - FPGA may force Neutral mode soon '
                                      '(check QCar LCD)' % (current_amps, amps_thresh, seconds_thresh))
                                overcurrent_tier_warned[i] = True
                        else:
                            overcurrent_tier_start[i] = None
                            overcurrent_tier_warned[i] = False

                    battery_v = float(car.batteryVoltage)
                    if battery_v < BATTERY_SHUTDOWN_VOLTAGE and battery_warn_state != 'shutdown':
                        print('WARNING: battery %.2fV below the %.1fV auto-shutdown threshold' %
                              (battery_v, BATTERY_SHUTDOWN_VOLTAGE))
                        battery_warn_state = 'shutdown'
                    elif (BATTERY_SHUTDOWN_VOLTAGE <= battery_v < BATTERY_WARN_VOLTAGE
                            and battery_warn_state is None):
                        print('WARNING: battery %.2fV below the %.1fV LOW BAT threshold' %
                              (battery_v, BATTERY_WARN_VOLTAGE))
                        battery_warn_state = 'warn'
                    elif battery_v >= BATTERY_WARN_VOLTAGE:
                        battery_warn_state = None

                    if abs(target_linear) < 1e-3:
                        desired_throttle = 0.0
                    elif target_linear > 0:
                        desired_throttle = THROTTLE_DEADBAND + THROTTLE_GAIN * target_linear
                    else:
                        desired_throttle = -(REVERSE_DEADBAND + REVERSE_GAIN * abs(target_linear))
                    desired_throttle = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, desired_throttle))
                    max_step = THROTTLE_RATE_LIMIT * dt
                    delta = max(-max_step, min(max_step, desired_throttle - current_throttle))
                    current_throttle += delta

                    # STEERING_RATE_LIMIT backstop (see comment above) - same pattern as throttle
                    # above, applied to the kinematic (untrimmed) angle so a hard left/right flip
                    # in target_steering gets damped into a bounded sweep instead of an instant
                    # full-range servo reversal.
                    max_steer_step = STEERING_RATE_LIMIT * dt
                    steer_delta = max(-max_steer_step, min(max_steer_step, target_steering - current_steering))
                    current_steering += steer_delta

                    leds = [0, 0, 0, 0, 0, 0, 1, 1]
                    if current_steering > 0.15:
                        leds[0] = 1
                        leds[2] = 1
                    elif current_steering < -0.15:
                        leds[1] = 1
                        leds[3] = 1
                    if current_throttle < 0:
                        leds[5] = 1

                    car.write(current_throttle, apply_trim(current_steering), leds)

                    # Odometry uses the untrimmed/kinematic angle - see compute_steering()'s
                    # docstring for why - and current_steering specifically (not target_steering),
                    # since STEERING_RATE_LIMIT means the two can genuinely differ: using the
                    # instantaneous target here would tell odometry the wheel turned further/
                    # faster than what was actually sent to the servo this cycle.
                    odom_data = odom.update(float(car.motorEncoder[0]), current_steering, dt)
                    # Capture-time timestamp, not send-time - see this file's module docstring for
                    # why this matters.
                    odom_data['t'] = time.time()
                    try:
                        conn.sendall((json.dumps(odom_data) + '\n').encode('utf-8'))
                    except OSError:
                        print('dev PC relay disconnected (send failed)')
                        break
            finally:
                conn.close()
                # Lost the relay connection - stop immediately rather than waiting out
                # CMD_TIMEOUT on the next accept() loop.
                target_linear = 0.0
                target_steering = 0.0
                current_throttle = 0.0
                current_steering = 0.0
                car.write(0.0, apply_trim(0.0), [0, 0, 0, 0, 0, 0, 0, 0])
    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.write(0.0, apply_trim(0.0), [0, 0, 0, 0, 0, 0, 0, 0])
        except Exception:
            pass
        car.terminate()
        server.close()


if __name__ == '__main__':
    main()
