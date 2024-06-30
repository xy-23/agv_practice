#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "agv_lidar_cpp/lidar.hpp"

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node
{
public:
  LidarNode() : Node("lidar_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&LidarNode::timer_callback, this));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  io::Lidar lidar_;

  void timer_callback()
  {
    auto finish = lidar_.scan();
    if (!finish) return;

    auto msg = sensor_msgs::msg::LaserScan();

    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_frame";
    msg.angle_min = lidar_.angle_min();
    msg.angle_max = lidar_.angle_max();
    msg.angle_increment = lidar_.angle_res();
    msg.time_increment = 1.0 / 108e3;
    msg.scan_time = 1.0 / lidar_.scan_frq();
    msg.range_min = 0.1;
    msg.range_max = 40.0;
    msg.ranges = lidar_.ranges();
    msg.intensities = lidar_.intensities();

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "published");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}