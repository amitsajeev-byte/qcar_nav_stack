#!/usr/bin/env python3
'''qcar_relay_node.py

Bridges /cmd_vel -> the QCar's onboard motor TCP server
(qcar_hardware/qcar_bridge.py), that same connection's odometry stream ->
/odom + odom->base TF, and the QCar's LiDAR TCP stream
(qcar_hardware/qcar_lidar_node.py) -> /scan.

Runs on the dev PC (Humble) - the only thing on this side of the link that
still speaks ROS2. Native ROS2 pub/sub between the QCar's onboard Dashing and
this machine's Humble doesn't interoperate (confirmed on hardware: raw UDP
passes fine in both directions on both arbitrary and DDS-discovery ports,
but Fast-RTPS discovery itself never completes - a protocol-version issue,
not a network one). This node exists to bridge across that gap over plain
TCP sockets instead. See the qcar_hardware package's README for the full
story of the onboard side.

The /odom + TF publishing exists for Nav2/AMCL, which looks up a live
odom->base transform at every scan callback - Cartographer (mapping)
doesn't need this, since it computes its own odometry internally from
/scan alone. This is open-loop wheel dead reckoning computed on the QCar
side (qcar_bridge.py) from encoder deltas + last-commanded steering angle,
not a measured one - see that file's docstring for the steering-trim
accuracy caveat this depends on.

Both qcar_bridge.py and qcar_lidar_node.py include their own QCar-side
capture timestamp ("t", time.time()) in every message rather than this
node stamping ROS2 messages at receipt time - receipt-time stamping,
across two independent TCP connections with uncorrelated network latency,
lets AMCL pair a given scan with an odom-derived pose from a slightly
different real-world instant, producing a rigid offset between the live
scan and the map. The QCar's clock and this machine's clock are NOT
synchronized (deliberately not fixed via system-level NTP/chrony, since
this is shared lab hardware and that would leave a persistent config
pointing at one user's personal dev PC), so qcar_clock_offset below
corrects for the measured per-session difference before using a QCar
timestamp as a ROS2 stamp.

Measuring qcar_clock_offset (repeat each session - offset can differ if
either machine reboots or its own NTP client re-syncs). Use a single
persistent SSH connection for all samples, not a fresh connection per
sample - a fresh connection's own handshake time gets folded into the
measurement asymmetrically and can inflate the apparent offset by an
order of magnitude:
    ssh -o ControlMaster=yes -o ControlPath=/tmp/qcar_ssh -o ControlPersist=60s -fN nvidia@<qcar_ip>
    for i in 1 2 3 4 5; do
      t1=$(date +%s.%N); remote=$(ssh -o ControlPath=/tmp/qcar_ssh nvidia@<qcar_ip> 'date +%s.%N'); t2=$(date +%s.%N)
      python3 -c "print(($remote) - (($t1 + $t2) / 2))"
    done
    ssh -o ControlPath=/tmp/qcar_ssh -O exit nvidia@<qcar_ip>
Average the printed values (should be tightly clustered - large spread
means a bad measurement, redo it) and pass as qcar_clock_offset (QCar time
minus this machine's time - positive means the QCar's clock is ahead).

    ros2 run qcar_gazebo qcar_relay_node.py --ros-args -p qcar_ip:=<ip> -p qcar_clock_offset:=<offset>
'''
import json
import math
import socket
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

MOTOR_PORT = 5555
LIDAR_PORT = 5556
RECONNECT_DELAY = 2.0  # seconds
SOCKET_TIMEOUT = 2.0  # seconds - bounds how long a dead connection takes to notice


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def qcar_time_to_stamp(qcar_time, qcar_clock_offset):
    '''Converts a QCar-clock time.time() reading into a builtin_interfaces/Time
    on this machine's clock, correcting for the measured offset between the
    two machines' independent, unsynchronized clocks.'''
    corrected = qcar_time - qcar_clock_offset
    sec = int(corrected)
    nanosec = int(round((corrected - sec) * 1e9))
    return Time(sec=sec, nanosec=nanosec)


class QCarRelayNode(Node):

    def __init__(self):
        super().__init__('qcar_relay_node')

        self.declare_parameter('qcar_ip', '')
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('qcar_clock_offset', 0.0)
        qcar_ip = self.get_parameter('qcar_ip').value
        if not qcar_ip:
            raise RuntimeError(
                'qcar_ip parameter is required, e.g.: ros2 run qcar_gazebo '
                'qcar_relay_node.py --ros-args -p qcar_ip:=172.24.0.66')
        self.qcar_ip = qcar_ip
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.qcar_clock_offset = self.get_parameter('qcar_clock_offset').value
        if self.qcar_clock_offset == 0.0:
            self.get_logger().warn(
                'qcar_clock_offset is 0.0 (default/unmeasured) - /scan and /odom '
                'timestamps will be wrong unless the QCar and this machine happen '
                'to have perfectly synced clocks, which is unlikely. See this '
                "file's module docstring for the measurement procedure.")

        self._motor_sock = None
        self._motor_lock = threading.Lock()
        self._stop = False

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._motor_thread = threading.Thread(target=self._motor_connection_loop, daemon=True)
        self._lidar_thread = threading.Thread(target=self._lidar_connection_loop, daemon=True)
        self._motor_thread.start()
        self._lidar_thread.start()

        self.get_logger().info('qcar_relay_node connecting to %s (motor:%d, lidar:%d)'
                                % (qcar_ip, MOTOR_PORT, LIDAR_PORT))

    # --- /cmd_vel -> QCar motor bridge ---

    def cmd_vel_callback(self, msg):
        line = (json.dumps({'linear_x': msg.linear.x, 'angular_z': msg.angular.z}) + '\n')
        with self._motor_lock:
            if self._motor_sock is None:
                return
            try:
                self._motor_sock.sendall(line.encode('utf-8'))
            except OSError:
                self._close_motor_socket()

    def _motor_connection_loop(self):
        while not self._stop:
            try:
                sock = socket.create_connection((self.qcar_ip, MOTOR_PORT), timeout=5.0)
                sock.settimeout(SOCKET_TIMEOUT)
                with self._motor_lock:
                    self._motor_sock = sock
                self.get_logger().info('connected to QCar motor bridge')
                # qcar_bridge.py sends odometry back on this same connection
                # (TCP is full-duplex) - read and publish it, same
                # newline-delimited-JSON pattern as the LiDAR loop below.
                buf = b''
                while not self._stop:
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if chunk == b'':
                        break
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        if line.strip():
                            self._publish_odom(line)
            except OSError:
                pass
            self._close_motor_socket()
            if not self._stop:
                self.get_logger().warn('QCar motor bridge disconnected, retrying...')
                time.sleep(RECONNECT_DELAY)

    def _close_motor_socket(self):
        with self._motor_lock:
            if self._motor_sock is not None:
                try:
                    self._motor_sock.close()
                except OSError:
                    pass
                self._motor_sock = None

    def _publish_odom(self, line):
        try:
            data = json.loads(line)
            stamp = qcar_time_to_stamp(data['t'], self.qcar_clock_offset)
            qx, qy, qz, qw = yaw_to_quaternion(data['yaw'])

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = data['x']
            odom.pose.pose.position.y = data['y']
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = data['v']
            odom.twist.twist.angular.z = data['yaw_rate']
            self.odom_pub.publish(odom)

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = data['x']
            tf.transform.translation.y = data['y']
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)
        except (ValueError, KeyError, TypeError):
            return

    # --- QCar LiDAR bridge -> /scan ---

    def _lidar_connection_loop(self):
        while not self._stop:
            try:
                sock = socket.create_connection((self.qcar_ip, LIDAR_PORT), timeout=5.0)
                sock.settimeout(SOCKET_TIMEOUT)
                self.get_logger().info('connected to QCar LiDAR bridge')
                buf = b''
                while not self._stop:
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if chunk == b'':
                        break
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        if line.strip():
                            self._publish_scan(line)
            except OSError:
                pass
            if not self._stop:
                self.get_logger().warn('QCar LiDAR bridge disconnected, retrying...')
                time.sleep(RECONNECT_DELAY)

    def _publish_scan(self, line):
        try:
            data = json.loads(line)
            scan = LaserScan()
            scan.header.stamp = qcar_time_to_stamp(data['t'], self.qcar_clock_offset)
            scan.header.frame_id = self.frame_id
            scan.angle_min = data['angle_min']
            scan.angle_max = data['angle_max']
            scan.angle_increment = data['angle_increment']
            scan.range_min = data['range_min']
            scan.range_max = data['range_max']
            scan.ranges = [float(r) for r in data['ranges']]
            self.scan_pub.publish(scan)
        except (ValueError, KeyError, TypeError):
            return

    def destroy_node(self):
        self._stop = True
        self._close_motor_socket()
        super().destroy_node()


def main():
    rclpy.init()
    node = QCarRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
