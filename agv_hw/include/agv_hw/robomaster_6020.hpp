#ifndef AGV_HW__ROBOMASTER_6020_HPP
#define AGV_HW__ROBOMASTER_6020_HPP

#include <memory>
#include <mutex>

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"
#include "socketcan.hpp"

namespace agv_hw
{
class Robomaster6020 : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robomaster6020)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // TODO on_init()
  int motor_id_;
  std::string can_name_;
  canid_t rx_id_;

  std::shared_ptr<SocketCAN> can_;

  std::mutex mutex_;
  can_frame rx_frame_;

  double command_effort_;
  double state_effort_;
  double state_velocity_;
  double state_position_;
  double state_temperature_;

  void callback(const can_frame & frame);
};

}  // namespace agv_hw

#endif  // AGV_HW__ROBOMASTER_6020_HPP