import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import json_transformer as json
import socket
import numpy as np

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

    def data_acc(self):
        self.get_logger().info("Trying to receive lidar data...\n")
        data = b'\x02\x02\x02\x02\x00\x0a\x02\x31\x01\x46'
        self.send_data(data)
        msg = self.recv_data()
        self.get_logger().info(f"Received lidar data message: {msg}\n")

    def rospc_transformer(self, data):
        json_data = json(data)
        msg = PointCloud2()
        msg.header.frame_id = 'H1E0-03B_Lidar_Data'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set point cloud data properties
        msg.height = 1
        msg.width = len(data)
        msg.is_dense = True 
        msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))

        # Convert custom data to bytes and assign it to the message
        msg.data = data.astype(np.float32).tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info('Published lidar scan point cloud')
            
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