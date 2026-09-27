#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

# Must match urdf/qcar_model.xacro hub joint origins (0.12960 + 0.12765 m).
WHEELBASE = 0.25725

MSG = """
QCar Twist Teleop
-----------------
Driving:
   w : forward
   s : backward
   a : steer left
   d : steer right
   z : steer centre
   x : stop & centre

Speed control:
   q : increase speed
   e : decrease speed

CTRL+C to quit
"""


class QCarTwistTeleop(Node):
    def __init__(self):
        super().__init__('qcar_teleop_twist')

        self.drive_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Below ~0.25-0.3 m/s the real vehicle sits at a static-friction deadband and may not
        # move at all - see qcar_hardware's qcar_bridge.py THROTTLE_GAIN comment.
        self.speed = 0.3  # m/s
        self.steer_angle = 0.4  # desired steering angle, radians
        self.current_drive = 0.0
        self.current_steer_angle = 0.0

        print(MSG)
        print(f'Current speed: {self.speed:.2f} m/s')

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def publish_drive(self, speed, steer_angle):
        # cmd_vel.angular.z is a yaw rate, not a steering angle - invert the bicycle model here
        # so both the Gazebo drive plugin and the real QCar bridge recover the intended
        # steer_angle from linear.x/angular.z alone.
        yaw_rate = 0.0
        if abs(speed) > 1e-3:
            yaw_rate = speed * math.tan(steer_angle) / WHEELBASE
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = yaw_rate
        self.drive_pub.publish(msg)

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.current_drive = self.speed
                print(f'Forward  | speed: {self.speed:.2f} m/s')
            elif key == 's':
                self.current_drive = -self.speed
                print(f'Backward | speed: {self.speed:.2f} m/s')
            elif key == 'a':
                self.current_steer_angle = self.steer_angle
                print(f'Steer Left  | angle: {self.steer_angle:.2f} rad')
            elif key == 'd':
                self.current_steer_angle = -self.steer_angle
                print(f'Steer Right | angle: {self.steer_angle:.2f} rad')
            elif key == 'z':
                self.current_steer_angle = 0.0
                print('Steer Centre')
            elif key == 'x':
                self.current_drive = 0.0
                self.current_steer_angle = 0.0
                print('Stop & Centre')
            elif key == 'q':
                self.speed = min(self.speed + 0.05, 1.0)
                print(f'Speed increased: {self.speed:.2f} m/s')
            elif key == 'e':
                self.speed = max(self.speed - 0.05, 0.05)
                print(f'Speed decreased: {self.speed:.2f} m/s')
            elif key == '\x03':
                break

            self.publish_drive(self.current_drive, self.current_steer_angle)


def main():
    rclpy.init()
    node = QCarTwistTeleop()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
