#!/usr/bin/env python3
'''calibrate_steering.py

Interactive steering-trim calibration. Holds the steering servo at a fixed
commanded angle (throttle=0, motor off) for a few seconds so the physical
wheel angle can be visually checked against straight - run repeatedly with
different values until the wheels sit straight, then record that value as
STEERING_TRIM in qcar_bridge.py. Needed because this vehicle's steering
mechanical center can't be corrected physically (no adjustable linkage
available on this chassis) - see this package's README for the full context.

    python3 calibrate_steering.py <steering_command_rad>

e.g. python3 calibrate_steering.py -0.05
'''
import sys
import time

from pal.products.qcar import QCar, IS_PHYSICAL_QCAR

HOLD_TIME = 12.0  # seconds


def main():
    if len(sys.argv) != 2:
        print('usage: python3 calibrate_steering.py <steering_command_rad>')
        sys.exit(1)
    steer = float(sys.argv[1])

    if not IS_PHYSICAL_QCAR:
        import qlabs_setup
        qlabs_setup.setup()

    print('Holding steering at %.4f rad for %.1fs (throttle=0)...' % (steer, HOLD_TIME))
    with QCar(readMode=1, frequency=50) as car:
        t0 = time.time()
        while time.time() - t0 < HOLD_TIME:
            car.read()
            car.write(0.0, steer, [0, 0, 0, 0, 0, 0, 1, 1])
            time.sleep(0.02)
    print('done')


if __name__ == '__main__':
    main()
