import math
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan


class TCPNode(Node):
    def __init__(self, host="192.168.192.101", port=2111):
        super().__init__("tcp_node")

        self.publisher = self.create_publisher(LaserScan, "scan", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

        self.min_degrees = -45
        self.max_degrees = 45

        self.login()
        self.set_frq_res(30, 0.1)
        self.set_degrees(self.min_degrees, self.max_degrees)
        self.start()

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def send(self, data: bytes):
        """blocking until data sent"""
        total_sent = 0

        while total_sent < len(data):
            sent = self.sock.send(data[total_sent:])
            if sent == 0:
                raise RuntimeError("Failed to send!")
            total_sent += sent

    def recv(self, size) -> bytes:
        """blocking until size reached"""
        received = b""

        while len(received) < size:
            data = self.sock.recv(size - len(received))
            if data == b"":
                raise RuntimeError("Failed to receive!")
            received += data

        return received

    def login(self):
        SUCCESS = b"\x02\x02\x02\x02\x00\x0A\x12\x01\x01\x26"

        self.send(b"\x02\x02\x02\x02\x00\x0E\x02\x01\x03\xF4\x72\x47\x44\x0D")
        reply = self.recv(len(SUCCESS))

        if reply == SUCCESS:
            self.get_logger().info("Login succeed.")
        else:
            raise RuntimeError(f"Failed to login: {reply.hex()}")

    def logout(self):
        pass

    def set_frq_res(self, scan_frq, degree_res):
        request = (
            b"\x02\x02\x02\x02"+ b"\x00\x0D" + b"\x01" + b"\x19"
            + struct.pack(">H", int(scan_frq * 100))
            + struct.pack(">H", int(degree_res * 10000))
        )

        checksum = struct.pack(">B", sum(request) & 0xFF)
        request += checksum

        self.send(request)
        reply = self.recv(len(request))
        self.get_logger().info(f"reque of set_frq_res: {request.hex()}")
        self.get_logger().info(f"Reply of set_frq_res: {reply.hex()}")

    def set_degrees(self, start, end):
        start += 90
        end += 90

        request = (
            b"\x02\x02\x02\x02\x00\x12\x01\x1B\x01"
            + struct.pack(">i", int(start * 10000))
            + struct.pack(">i", int(end * 10000))
        )

        checksum = struct.pack(">B", sum(request) & 0xFF)
        request += checksum

        self.send(request)
        reply = self.recv(len(request))
        self.get_logger().info(f"reque of set_frq_res: {request.hex()}")
        self.get_logger().info(f"Reply of set_degrees: {reply.hex()}")

    def start(self):
        SUCCESS = b"\x02\x02\x02\x02\x00\x0A\x12\x31\x01\x56"

        self.send(b"\x02\x02\x02\x02\x00\x0a\x02\x31\x01\x46")
        reply = self.recv(len(SUCCESS))

        if reply == SUCCESS:
            self.get_logger().info("Start succeed.")
        else:
            raise RuntimeError(f"Failed to start: {reply.hex()}")

    def timer_callback(self):
        try:
            self.recv_loop()
        except Exception as e:
            self.get_logger().warn(f'Failed in recv_loop: {e}')

    def recv_loop(self):
        self.wait_header()

        length_raw = self.recv(2)
        length = struct.unpack(">H", length_raw)[0]
        data = self.recv(length - 6)  # header + length = 6

        total = b"\x02\x02\x02\x02" + length_raw + data
        checksum = sum(total[:-1]) & 0xFF
        if total[-1] != checksum:
            raise RuntimeError(f"Failed to checksum!")

        op, cmd, info, scan_cnt, frame_i, scan_frq, angle_res, N, i, n = struct.unpack(
            ">BBBHBHHHHH", data[:16]
        )

        scan_frq /= 1e2
        angle_res = math.radians(angle_res / 1e4)
        scans = struct.unpack(">" + "H" * 2 * n, data[16:-11])

        if frame_i == 0:
            self.last_i = -1
            self.point_cnt = 0
            self.ranges = []
            self.intensities = []
            self.stamp = self.get_clock().now().to_msg()

        if frame_i != self.last_i + 1:
            return
        
        self.last_i = frame_i
        self.point_cnt += n
        self.ranges += [scans[i] / 1e3 for i in range(0, len(scans), 2)]
        self.intensities += [scans[i] / 1.0 for i in range(1, len(scans), 2)]

        if self.point_cnt == N:
            lidar_msg = LaserScan()
            lidar_msg.header.stamp = self.stamp
            lidar_msg.header.frame_id = "laser_frame"
            lidar_msg.angle_increment = angle_res
            lidar_msg.angle_min = math.radians(self.min_degrees)
            lidar_msg.angle_max = angle_res * (N - 1) + math.radians(self.min_degrees)
            lidar_msg.scan_time = 1 / scan_frq
            lidar_msg.time_increment = 1 / 108e3
            lidar_msg.range_min = 0.1
            lidar_msg.range_max = 40.0
            lidar_msg.ranges = self.ranges
            lidar_msg.intensities = self.intensities
            self.publisher.publish(lidar_msg)
            self.get_logger().info("Publish succeed.")

    def wait_header(self):
        count = 0
        while count < 4:
            data = self.recv(1)
            if data == b"\x02":
                count += 1
            else:
                count = 0


def main(args=None):
    rclpy.init(args=args)
    tcp_node = TCPNode()

    try:
        rclpy.spin(tcp_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tcp_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
