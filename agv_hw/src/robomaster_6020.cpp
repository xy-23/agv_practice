#include "agv_hw/robomaster_6020.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_hw
{

hardware_interface::CallbackReturn Robomaster6020::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用父类方法，将info赋值给属性info_
  // info是在urdf文件的<ros2_control>标签中定义的信息
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 检查是否只有一个joint
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("Robomaster6020"), "Expected single joint!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];

  // 检查joint是否只有一个command_interface
  if (joint.command_interfaces.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("Robomaster6020"), "Expected single command_interface!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 检查command_interface是否为effort（扭矩）
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
    RCLCPP_FATAL(
      rclcpp::get_logger("Robomaster6020"), "Expected \"%s\" command_interface!",
      hardware_interface::HW_IF_EFFORT);
    return hardware_interface::CallbackReturn::ERROR;
  }

  /// 检查state_interfaces是否包含effort、velocity、position、temperature

  bool state_interfaces_ok = true;
  std::vector<int> required_count_list{0, 0, 0, 0};
  std::vector<std::string> required_name_list{
    hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_POSITION, "temperature"};

  for (const auto & state_interface : joint.state_interfaces) {
    for (std::size_t i = 0; i < required_name_list.size(); i++) {
      if (state_interface.name == required_name_list[i]) required_count_list[i]++;
    }
  }

  for (std::size_t i = 0; i < required_name_list.size(); i++) {
    if (required_count_list[i] == 1) continue;
    RCLCPP_FATAL(
      rclcpp::get_logger("Robomaster6020"), "Expected single \"%s\" state_interface!",
      required_name_list[i].c_str());
    state_interfaces_ok = false;
  }

  if (!state_interfaces_ok) return hardware_interface::CallbackReturn::ERROR;

  // 检查通过！
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Robomaster6020::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 配置电机时将属性清零
  command_effort_ = 0;
  state_effort_ = 0;
  state_velocity_ = 0;
  state_position_ = 0;
  state_temperature_ = 0;

  // 打开CAN
  can_ = std::make_shared<SocketCAN>(
    can_name_, std::bind(&Robomaster6020::callback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Robomaster6020::export_state_interfaces()
{
  // 向外部导出状态接口（state interfaces），本质是指针

  auto joint_name = info_.joints[0].name;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_name, hardware_interface::HW_IF_EFFORT, &state_effort_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_name, hardware_interface::HW_IF_VELOCITY, &state_velocity_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_name, hardware_interface::HW_IF_POSITION, &state_position_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(joint_name, "temperature", &state_temperature_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Robomaster6020::export_command_interfaces()
{
  // 向外部导出指令接口（command interfaces），本质是指针

  auto joint_name = info_.joints[0].name;
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_name, hardware_interface::HW_IF_EFFORT, &command_effort_));

  return command_interfaces;
}

hardware_interface::CallbackReturn Robomaster6020::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 在启动时command应该等于state
  command_effort_ = state_effort_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Robomaster6020::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Robomaster6020::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lg(mutex_);

  uint16_t ecd = (rx_frame_.data[0] << 8) | rx_frame_.data[1];
  int16_t rpm = (rx_frame_.data[2] << 8) | rx_frame_.data[3];
  int16_t current = (rx_frame_.data[4] << 8) | rx_frame_.data[5];

  state_position_ = static_cast<double>(ecd) / 8192 * 2 * M_PI;  // rad
  state_velocity_ = static_cast<double>(rpm) / 60 * 2 * M_PI;    // rad/s
  state_effort_ = static_cast<double>(current) / 0.741;          // N.m
  state_temperature_ = static_cast<double>(rx_frame_.data[6]);   // C

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Robomaster6020::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int16_t current = command_effort_ * 0.741 / 3 * 16384;

  can_frame tx_frame;
  tx_frame.can_id = 0x1FF;  // TODO
  tx_frame.len = 8;
  tx_frame.data[0] = current >> 8;
  tx_frame.data[1] = current;

  can_->write(&tx_frame);

  return hardware_interface::return_type::OK;
}

void Robomaster6020::callback(const can_frame & frame)
{
  if (frame.can_id != rx_id_) return;
  std::lock_guard<std::mutex> lg(mutex_);
  rx_frame_ = frame;
}

}  // namespace agv_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(agv_hw::Robomaster6020, hardware_interface::ActuatorInterface)