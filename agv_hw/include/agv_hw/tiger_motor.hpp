#ifndef TIGER_MOTOR_HPP
#define TIGER_MOTOR_HPP

#include <memory>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "socketcan.hpp"

//电机个数
#define MotorUseNumber 2
//左右两电机ID
#define CANID_LEFT 1
#define CANID_RIGHT 2

//初始化时数据
#define CMD_SET_SPEED 0X23  //速度设定
#define INDEX_SPEED 0X2341
#define SUBINDEX_SPEED 0x00

//电机速度与rpm的速度系数
#define Com_Encoder1Resolution 4096

typedef enum _PDOInitState {
  PDO_NEEDINIT = 0,    //马达PDO需要初始化
  PDO_INITING = 1,     //马达PDO初始化中
  PDO_INITFINISH = 2,  //马达初始化完成
} PDOInitState;

typedef enum _StatusInitState {
  Status_NEEDINIT = 0,   //需要检查状态字
  Status_INITING = 1,    //状态字检查中
  Status_INITFINISH = 2  //驱动器 已就绪
} StatusInitState;

namespace agv_hw
{
class Tiger_motor : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Tiger_motor)

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
  unsigned short subIndex, Status;

  PDOInitState mCurrentMotorPDOState = PDO_NEEDINIT;           //初始化PDO的标志位
  StatusInitState mCurrentMotorStatusState = Status_NEEDINIT;  //初始化获取状态字标志位

  uint8_t mPDOSetMotorNo = 1;
  uint8_t mPDOSetMotorStep = 0;
  uint8_t mGetStatusMotorNo = 1, mGetStatusMotorStep = 0;

  unsigned char SendChannel = 0;

  bool motor_inited_;
  unsigned int motor_id_left_;
  unsigned int motor_id_right_;
  std::string can_name_;
  canid_t tx_id_left;
  canid_t tx_id_right;

  std::shared_ptr<SocketCAN> can_;

  std::mutex mutex_;
  can_frame rx_frame_;

  double left_command_speed_ = 0;   //m/s
  double right_command_speed_ = 0;  //m/s
  double left_state_position_ = 0;
  double right_state_position_ = 0;
  int left_state_position_mid;
  int right_state_position_mid;
  void callback(const can_frame & frame);
  void SendSetPDOCmd(
    can_frame * tx_frame, unsigned short canID, unsigned short addr1, unsigned char subAddr,
    unsigned int data, unsigned char size);
  void start_up_NMT(can_frame * tx_frame, uint8_t canID);
};

}  // namespace agv_hw

#endif  // AGV_HW__ROBOMASTER_6020_HPP