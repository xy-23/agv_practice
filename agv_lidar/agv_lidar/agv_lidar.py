import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json_transformer as json
import socket

class TCPNode(Node):
    def __init__(self, ip_config = '192.168.192.101', port_config = 2111):
        super().__init__('tcp_node')
        #TCP connect
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = ip_config
        port = port_config
        addr = (host, port)
        self.tcp_client.connect(addr)

        self.login()
        self.data_acc()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.data_acc()

    def send_data(self, data):
        self.tcp_client.send(data)

    def recv_data(self, buffer_size=1024):
        return self.tcp_client.recv(buffer_size).hex()

    def login(self):
        self.get_logger().info("Trying to log in...\n")
        data = b'\x02\x02\x02\x02\x00\x0E\x02\x01\x03\xF4\x72\x47\x44\x0D'
        self.send_data(data)
        msg = self.recv_data()
        self.get_logger().info(f"Received log in message: {msg}\n")

    def logout(self):
        self.get_logger().info("Trying to log out...\n")
        data = b'\x02\x02\x02\x02\x00\x09\x02\x02\x15'
        self.send_data(data)
        msg = self.recv_data()
        self.get_logger().info(f"Received log out message: {msg}\n")

    def rospc_transformer(self, data):
        json_data, [rangelist, relist] = json(data)
        angle_min = -45
        angle_max = 225

        lidar_msg = LaserScan()
        lidar_msg.angle_min = angle_min
        lidar_msg.angle_max = angle_max
        lidar_msg.angle_increment = 0.0
        lidar_msg.scan_time = 0.0
        lidar_msg.range_min = min(rangelist)
        lidar_msg.range_max = max(rangelist)
        lidar_msg.ranges = rangelist
        lidar_msg.intensities = relist

        return lidar_msg

    def data_acc(self):
        self.get_logger().info("Trying to receive lidar data...\n")
        data = b'\x02\x02\x02\x02\x00\x0a\x02\x31\x01\x46'
        self.send_data(data)
        msg = self.recv_data()
        msg = self.rospc_transformer(msg)
        self.get_logger().info(f"Received lidar data message: {msg}\n")

    def shutdown(self):
        self.logout()
        self.tcp_client.close()
        self.get_logger().info("Lidar data node shut down.\n")

def main(args = None):
    rclpy.init(args=args)
    tcp_node = TCPNode()
    rclpy.spin(tcp_node)
    tcp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()