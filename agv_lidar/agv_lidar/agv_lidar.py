import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from agv_lidar.json_transformer import Json_transformer
import socket
import struct
import math


class TCPNode(Node):
    def __init__(self, ip_config="192.168.192.101", port_config=2111):
        super().__init__("tcp_node")
        self.publisher_ = self.create_publisher(LaserScan, "agv_lidar", 10)
        # TCP connect
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = ip_config
        port = port_config
        addr = (host, port)
        self.tcp_client.connect(addr)

        self.login()
        self.start_recv = False

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.start_recv:
            self.acc_init()
            self.start_recv = True

        header = self.tcp_client.recv(4)

        if header.hex() != "02020202":
            return

        length = int(self.tcp_client.recv(2).hex(), 16)
        data = self.tcp_client.recv(length - 6)

        exe, cmd, info, scan_cnt, frame_i, scan_f, angle_res, N, i, n = struct.unpack(
            ">BBBHBHHHHH", data[:16]
        )

        scan_f /= 1e2
        angle_res = math.radians(angle_res / 1e4)

        MAX_ANGLE = math.radians(135)
        MIN_ANGLE = math.radians(-135)

        scans = struct.unpack(">" + "I" * n * 2, data[16:-11])

        ranges = [scans[i] / 1e3 for i in range(0, len(data), 2)]
        intensities = [scans[i] for i in range(1, len(data), 2)]

        lidar_msg = LaserScan()
        lidar_msg.header.stamp = self.get_clock().now().to_msg()
        lidar_msg.header.frame_id = "laser_frame"
        lidar_msg.angle_increment = (MAX_ANGLE - MIN_ANGLE) / N
        lidar_msg.angle_min = lidar_msg.angle_increment * i
        lidar_msg.angle_max = lidar_msg.angle_increment * (i + n)
        # lidar_msg.scan_time = 0.033
        # lidar_msg.time_increment = 1/108e3
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 40.0
        lidar_msg.ranges = ranges
        lidar_msg.intensities = intensities
        self.publisher_.publish(lidar_msg)

    def send_data(self, data):
        self.tcp_client.send(data)

    def recv_data(self, buffer_size=1024):
        return self.tcp_client.recv(buffer_size).hex()

    def login(self):
        self.get_logger().info("Trying to log in...\n")
        data = b"\x02\x02\x02\x02\x00\x0E\x02\x01\x03\xF4\x72\x47\x44\x0D"
        self.send_data(data)
        msg = self.recv_data()
        self.get_logger().info(f"Received log in message: {msg}\n")

    def logout(self):
        self.get_logger().info("Trying to log out...\n")
        data = b"\x02\x02\x02\x02\x00\x09\x02\x02\x15"
        self.send_data(data)
        msg = self.recv_data()
        self.get_logger().info(f"Received log out message: {msg}\n")

    def rospc_transformer(self, data):
        json_data, [rangelist, relist] = Json_transformer(data)
        angle_min = -135.0 / 57.3
        angle_max = 135.0 / 57.3

        lidar_msg = LaserScan()
        lidar_msg.header.stamp = self.get_clock().now().to_msg()
        lidar_msg.header.frame_id = "laser_frame"
        lidar_msg.angle_min = angle_min
        lidar_msg.angle_max = angle_max
        lidar_msg.angle_increment = 270 / len(rangelist) / 57.3
        # lidar_msg.scan_time = 0.033
        # lidar_msg.time_increment = 1/108e3
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 40.0
        lidar_msg.ranges = rangelist
        lidar_msg.intensities = relist

        return lidar_msg

    def acc_init(self):
        self.get_logger().info("Trying to receive lidar data...\n")
        data = b"\x02\x02\x02\x02\x00\x0a\x02\x31\x01\x46"
        self.send_data(data)
        self.recv_data()

    def data_acc(self):
        msg = self.recv_data()
        msg = self.rospc_transformer(msg)
        self.publisher_.publish(msg)

    def shutdown(self):
        self.logout()
        self.tcp_client.close()
        self.get_logger().info("Lidar data node shut down.\n")


def main(args=None):
    rclpy.init(args=args)
    tcp_node = TCPNode()
    rclpy.spin(tcp_node)
    tcp_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
