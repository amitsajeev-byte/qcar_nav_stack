#!/usr/bin/env python3
'''calibrate_throttle.py

Standalone throttle-duty calibration sweep, forward or reverse. Directly
drives a fixed PWM duty (bypassing qcar_bridge.py's cmd_vel/gain path
entirely - same direct-HAL pattern as calibrate_steering.py) for each duty
value in the sweep, measures the real steady-state speed from encoder
deltas, then fits duty = DEADBAND + GAIN*speed via least squares (duty as
the dependent variable, since that's the form qcar_bridge.py's throttle
formula actually needs at runtime: desired_throttle = DEADBAND +
GAIN*abs(target_linear)).

Run with qcar_bridge.py STOPPED first - needs exclusive HAL access, same
requirement as calibrate_steering.py.

    python3 calibrate_throttle.py fwd <confirmed_clear_distance_m>
    python3 calibrate_throttle.py rev <confirmed_clear_distance_m>
    python3 calibrate_throttle.py rev <confirmed_clear_distance_m> 0.05 0.06 0.07   # custom sweep

<confirmed_clear_distance_m> is not optional - it's the actual clear space just confirmed with
the operator in the direction being tested, right before running this. The script uses half of
it (capped at DEFAULT_DISTANCE_BUDGET_M) as a hard cumulative-distance abort budget and zeroes
throttle immediately if exceeded - see DistanceBudget below for why this exists.

Forward and reverse do not reliably share one DEADBAND/GAIN pair - real testing has found reverse
noticeably more sensitive to battery state and floor condition than forward. Run both directions
separately (~10 min total for the default 4-point sweep each) and compare the fitted DEADBAND/GAIN
against whatever is currently hardcoded in qcar_bridge.py, updating it if they've drifted
meaningfully. The underlying stiction/gain physics are battery- and floor-condition-dependent, so
treat this as a periodic recalibration, not a one-time setup step.
'''
import sys
import time

from pal.products.qcar import QCar, IS_PHYSICAL_QCAR
# Imported, not duplicated, so this can't silently drift from qcar_bridge.py's calibrated value.
# Without this, car.write()'s raw steering=0.0 is NOT straight - per STEERING_TRIM's own
# calibration data, kinematic 0.0 corresponds to a real wheel angle of about +0.11 rad off-center.
from qcar_bridge import STEERING_TRIM

METERS_PER_COUNT = 0.01977 / 2880.0  # matches qcar_bridge.py
THROTTLE_LIMIT = 0.3  # matches qcar_bridge.py's safety cap - sweep values must stay under this

SETTLE_TIME = 1.5    # seconds to let it reach steady state before measuring
MEASURE_TIME = 1.5   # seconds of steady-state window to average speed over
SAMPLE_DT = 0.02      # seconds - matches qcar_bridge.py's 50Hz control loop
REST_SETTLE_SPEED = 0.01   # m/s - below this counts as "stopped" between steps
REST_SETTLE_HOLD = 0.3     # seconds the speed must stay under threshold to call it stopped
REST_MAX_TIME = 6.0        # safety cap - documented coast time is ~2.1-2.5s, this has margin

# A duty this small still commands real, sometimes surprising speed (0.08 has previously produced
# an untested ~1.25 m/s when a wider sweep was tried) - don't extend this range without a specific
# reason, and even then only with a lot of confirmed clear space. This default spans the
# ~0.2-0.6 m/s zone normal teleop/nav2 operation actually uses.
DEFAULT_DUTY_SWEEP = [0.05, 0.06, 0.07, 0.08]

# Hard safety cutoff, independent of the duty cap above - belt and suspenders. Tracks distance
# traveled (summed absolute value, both directions) across the WHOLE run and aborts immediately
# (zero throttle) the moment it's exceeded, rather than trusting the operator's timing/attention
# alone. Set this from the ACTUAL confirmed-clear distance each time this script is run, with
# real margin - never assume a distance is still clear from an earlier session or a different
# spot. Passed as sys.argv[-1] appended after the duty list, or defaults conservatively low.
DEFAULT_DISTANCE_BUDGET_M = 1.5


class DistanceBudget:
    '''Hard safety cutoff, independent of the duty cap - tracks cumulative distance traveled
    (both directions summed) across the whole run and flags exceeded once past limit_m, so
    callers can immediately zero throttle rather than trusting the operator's timing/attention
    alone. See DEFAULT_DISTANCE_BUDGET_M comment for why this exists.'''

    def __init__(self, limit_m):
        self.limit_m = limit_m
        self.traveled_m = 0.0
        self.exceeded = False

    def add(self, delta_m):
        self.traveled_m += abs(delta_m)
        if self.traveled_m > self.limit_m:
            self.exceeded = True
        return self.exceeded


def _read_speed(car, last_encoder, last_t, now):
    encoder = float(car.motorEncoder[0])
    speed = 0.0
    delta_m = 0.0
    if last_encoder is not None and last_t is not None:
        dt = now - last_t
        delta_m = (encoder - last_encoder) * METERS_PER_COUNT
        if dt > 0:
            speed = delta_m / dt
    return encoder, speed, delta_m


def drive_and_measure(car, signed_duty, budget):
    '''Holds signed_duty for SETTLE_TIME+MEASURE_TIME seconds, returns the
    mean signed speed (m/s) measured over the last MEASURE_TIME seconds -
    or None if budget.exceeded fired mid-step (throttle already zeroed).'''
    last_encoder = None
    last_t = None
    speeds = []
    t0 = time.time()
    while time.time() - t0 < SETTLE_TIME + MEASURE_TIME:
        car.read()
        car.write(signed_duty, STEERING_TRIM, [0, 0, 0, 0, 0, 0, 1, 1])
        now = time.time()
        encoder, speed, delta_m = _read_speed(car, last_encoder, last_t, now)
        if budget.add(delta_m):
            car.write(0.0, STEERING_TRIM, [0, 0, 0, 0, 0, 0, 0, 0])
            print('  DISTANCE BUDGET EXCEEDED (%.2fm > %.2fm) - aborting, throttle zeroed'
                  % (budget.traveled_m, budget.limit_m))
            return None
        if (now - t0) > SETTLE_TIME:
            speeds.append(speed)
        last_encoder, last_t = encoder, now
        time.sleep(SAMPLE_DT)
    return sum(speeds) / len(speeds) if speeds else 0.0


def rest_until_stopped(car, budget):
    '''Zero throttle until real speed has stayed under REST_SETTLE_SPEED for
    REST_SETTLE_HOLD seconds straight, or REST_MAX_TIME elapses regardless
    (safety fallback) - a fixed rest timer isn't reliably longer than the
    documented ~2.1-2.5s coast time, and starting the next step's
    measurement with residual velocity (e.g. still coasting forward when a
    reverse-duty step begins) would contaminate that step's speed reading.'''
    last_encoder = None
    last_t = None
    stable_since = None
    t0 = time.time()
    while time.time() - t0 < REST_MAX_TIME:
        car.read()
        car.write(0.0, STEERING_TRIM, [0, 0, 0, 0, 0, 0, 0, 0])
        now = time.time()
        encoder, speed, delta_m = _read_speed(car, last_encoder, last_t, now)
        budget.add(delta_m)  # coasting still counts against the budget
        if last_encoder is not None:
            if abs(speed) < REST_SETTLE_SPEED:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since > REST_SETTLE_HOLD:
                    return
            else:
                stable_since = None
        last_encoder, last_t = encoder, now
        time.sleep(SAMPLE_DT)


def fit_deadband_gain(duties, speeds):
    '''OLS fit duty = DEADBAND + GAIN*speed (speed as X, duty as Y) - this
    is the form qcar_bridge.py's throttle formula actually needs, even
    though duty was the controlled/independent variable in this sweep.'''
    n = len(speeds)
    mean_x = sum(speeds) / n
    mean_y = sum(duties) / n
    cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(speeds, duties))
    var = sum((x - mean_x) ** 2 for x in speeds)
    if var < 1e-9:
        return None, None
    gain = cov / var
    deadband = mean_y - gain * mean_x
    return deadband, gain


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ('fwd', 'rev'):
        print('usage: python3 calibrate_throttle.py <fwd|rev> <confirmed_clear_distance_m> '
              '[duty1 duty2 ...]')
        print('confirmed_clear_distance_m: the ACTUAL clear space just confirmed with the '
              'operator, in the direction being tested - not a guess, not left over from an '
              'earlier session/position. The script uses a fraction of this as a hard abort '
              'budget - it does not just trust the duty cap.')
        sys.exit(1)
    direction = 1.0 if sys.argv[1] == 'fwd' else -1.0
    clear_distance_m = float(sys.argv[2])
    duty_sweep = [float(d) for d in sys.argv[3:]] if len(sys.argv) > 3 else DEFAULT_DUTY_SWEEP
    for d in duty_sweep:
        if abs(d) > THROTTLE_LIMIT:
            print('duty %.3f exceeds THROTTLE_LIMIT (%.2f) - refusing' % (d, THROTTLE_LIMIT))
            sys.exit(1)

    # Half the confirmed clear distance, capped at DEFAULT_DISTANCE_BUDGET_M - real margin
    # against the confirmed space, not the full amount of it.
    distance_budget_m = min(clear_distance_m * 0.5, DEFAULT_DISTANCE_BUDGET_M)

    if not IS_PHYSICAL_QCAR:
        import qlabs_setup
        qlabs_setup.setup()

    est_total = len(duty_sweep) * (SETTLE_TIME + MEASURE_TIME) + (len(duty_sweep) + 1) * 1.5
    print('Calibrating %s: duty sweep %s (~%.0fs), distance budget %.2fm (confirmed clear: %.2fm)'
          % (sys.argv[1], duty_sweep, est_total, distance_budget_m, clear_distance_m))

    results = []
    with QCar(readMode=0, frequency=50) as car:
        budget = DistanceBudget(distance_budget_m)
        rest_until_stopped(car, budget)
        for duty in duty_sweep:
            if budget.exceeded:
                break
            signed_duty = direction * duty
            speed = drive_and_measure(car, signed_duty, budget)
            if speed is None:
                break
            print('  duty=%+.3f -> measured speed=%+.4f m/s  (traveled so far: %.2fm)'
                  % (signed_duty, speed, budget.traveled_m))
            results.append((duty, abs(speed)))
            rest_until_stopped(car, budget)
        # Always end at zero regardless of how the loop above exited.
        car.read()
        car.write(0.0, STEERING_TRIM, [0, 0, 0, 0, 0, 0, 0, 0])

    if budget.exceeded:
        print('Stopped early - distance budget exceeded. Reposition and rerun if more points '
              'are needed.')

    duties = [d for d, s in results]
    speeds = [s for d, s in results]

    print()
    print('duty   measured_speed')
    for d, s in results:
        print('%.3f  %.4f' % (d, s))
    print()
    if len(results) < 2:
        print('Not enough points for a fit.')
        return
    deadband, gain = fit_deadband_gain(duties, speeds)
    if deadband is not None:
        print('Fitted: duty = %.4f + %.4f * speed  ->  DEADBAND=%.4f, GAIN=%.4f'
              % (deadband, gain, deadband, gain))
    else:
        print('Fit failed - measured speed identical across the whole sweep (car never moved?)')


if __name__ == '__main__':
    main()
