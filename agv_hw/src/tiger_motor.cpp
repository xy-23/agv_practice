
#include "agv_hw/tiger_motor.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_hw
{

hardware_interface::CallbackReturn Tiger_motor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用父类方法，将info赋值给属性info_
  // info是在urdf文件的<ros2_control>标签中定义的信息
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 读取参数
  can_name_ = info_.hardware_parameters["can_name"];
  motor_id_left_ = stod(info_.hardware_parameters["motor_id_left"]);
  tx_id_left = 0x600 + motor_id_left_;
  motor_id_right_ = stod(info_.hardware_parameters["motor_id_right"]);
  tx_id_right = 0x600 + motor_id_right_;

  // // 检查是否只有一个joint
  // if (info_.joints.size() != 2) {
  //   RCLCPP_FATAL(rclcpp::get_logger("Tiger_motor"), "Expected Two joint!");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // const auto & joint = info_.joints[0];

  // // 检查joint是否只有一个command_interface
  // if (joint.command_interfaces.size() != 2) {
  //   RCLCPP_FATAL(rclcpp::get_logger("Tiger_motor"), "Expected single command_interface!");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // // 检查command_interface是否为speed
  // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
  //   RCLCPP_FATAL(
  //     rclcpp::get_logger("Tiger_motor"), "Expected \"%s\" command_interface!",
  //     hardware_interface::HW_IF_VELOCITY);
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
  //   RCLCPP_FATAL(
  //     rclcpp::get_logger("Tiger_motor"), "Expected \"%s\" command_interface!",
  //     hardware_interface::HW_IF_VELOCITY);
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  /// 检查state_interfaces是否包含position

  // bool state_interfaces_ok = true;
  // std::vector<int> required_count_list{0, 0, 0, 0};
  // std::vector<std::string> required_name_list{
  //   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION};

  // for (const auto & state_interface : joint.state_interfaces) {
  //   for (std::size_t i = 0; i < required_name_list.size(); i++) {
  //     if (state_interface.name == required_name_list[i]) required_count_list[i]++;
  //   }
  // }

  // for (std::size_t i = 0; i < required_name_list.size(); i++) {
  //   if (required_count_list[i] == 1) continue;
  //   if (required_count_list[i] > 1) {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Tiger_motor"), "Expected single \"%s\" state_interface!",
  //       required_name_list[i].c_str());
  //   } else {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Tiger_motor"), "Not found \"%s\" state_interface!",
  //       required_name_list[i].c_str());
  //   }
  //   state_interfaces_ok = false;
  // }

  // if (!state_interfaces_ok) return hardware_interface::CallbackReturn::ERROR;

  motor_inited_ = false;

  // 检查通过！
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tiger_motor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 配置电机时将属性清零
  left_command_speed_ = 0;
  right_command_speed_ = 0;
  left_state_position_ = 0;
  right_state_position_ = 0;

  // 打开CAN
  can_ = std::make_shared<SocketCAN>(
    can_name_, std::bind(&Tiger_motor::callback, this, std::placeholders::_1));

  can_frame tx_frame;
  while (mCurrentMotorPDOState != PDO_INITFINISH) {
    // RCLCPP_INFO(
    //   rclcpp::get_logger("Tiger_motor"), "Motor %d %d %d", mPDOSetMotorStep, mGetStatusMotorStep,
    //   mCurrentMotorStatusState);

    if (mCurrentMotorStatusState == Status_INITFINISH)  //驱动器已就绪
    {
      //发PDO配置
      static float timeCount = 0;
      if (mCurrentMotorPDOState == PDO_NEEDINIT) {
        mPDOSetMotorNo = 1;
        mPDOSetMotorStep = 0;
        mCurrentMotorPDOState = PDO_INITING;
      }
      timeCount += 0.001;
      if (timeCount > 0.05f) {
        timeCount = 0;
        if (mPDOSetMotorStep > 9)  //步骤完成，设置马达PDO成功
        {
          mPDOSetMotorStep = 0;
          mPDOSetMotorNo += 1;
          if (mPDOSetMotorNo > MotorUseNumber)  //设置所有马达PDO完成
          {
            mCurrentMotorPDOState = PDO_INITFINISH;  //初始化完成标志
          }
        }
        switch (mPDOSetMotorStep) {
          case 0:  //关闭同步发生器
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1005, 0x00, 0x00000080, 4);
            break;
          case 1:  //设置同步消息时间  本例程设置10ms同步消息
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1006, 0x00, 0x00000064, 4);
            break;
          case 2:  //开启同步发生器
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1005, 0x00, 0x40000080, 4);
            break;
          case 3:  //禁用TPDO1
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1a01, 0x00, 0, 1);
            break;
          case 4:  //设置通讯参数为同步模式3SYNC
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1801, 0x02, 1, 1);
            break;
          case 5:  //映射对象  实际位置
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1a01, 0x01, 0x60640020, 4);
            break;
          case 6:  //映射对象  实际电压
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1a01, 0x02, 0x22010010, 4);
            break;
          case 7:  //映射对象  实际电流
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1a01, 0x03, 0x221C0010, 4);
            break;
          case 8:  //启用TPDO0
            Tiger_motor::SendSetPDOCmd(&tx_frame, mPDOSetMotorNo, 0x1a01, 0x00, 3, 1);
            break;
          case 9:
            Tiger_motor::start_up_NMT(&tx_frame, mPDOSetMotorNo);
            mPDOSetMotorStep += 1;
            break;
          default:
            break;
        }
      }
    } else {
      //查状态字
      static float timeCount = 0;
      if (mCurrentMotorStatusState == Status_NEEDINIT) {
        mGetStatusMotorNo = 1;
        mGetStatusMotorStep = 0;
        mCurrentMotorStatusState = Status_INITING;
      }
      timeCount += 0.001;
      if (timeCount > 0.05f) {
        timeCount = 0;

        if (mGetStatusMotorStep > 0)  //步骤完成，电机已经就绪
        {
          mGetStatusMotorStep = 0;
          mGetStatusMotorNo += 1;
          if (mGetStatusMotorNo > MotorUseNumber)  //所有马达就绪
          {
            mCurrentMotorStatusState = Status_INITFINISH;  //初始化完成标志
          }
        }
        switch (mGetStatusMotorStep) {
          case 0:  //复位
                   //查状态字
            tx_frame.can_id = 0x600 + mGetStatusMotorNo;
            tx_frame.len = 8;
            tx_frame.data[0] = 0x40;
            tx_frame.data[1] = 0x6041 & 0xff;
            tx_frame.data[2] = (0x6041 >> 8) & 0xff;
            for (uint8_t i = 3; i < 8; i++) {
              tx_frame.data[i] = 0;
            }
            can_->write(&tx_frame);
            break;

          default:
            break;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  motor_inited_ = true;
  RCLCPP_INFO(rclcpp::get_logger("Tiger_motor"), "Motor inited!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Tiger_motor::export_state_interfaces()
{
  // 向外部导出状态接口（state interfaces），本质是指针

  auto joint_name_left = info_.joints[0].name;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_name_left, hardware_interface::HW_IF_POSITION, &left_state_position_));

  auto joint_name_right = info_.joints[1].name;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_name_right, hardware_interface::HW_IF_POSITION, &right_state_position_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Tiger_motor::export_command_interfaces()
{
  // 向外部导出指令接口（command interfaces），本质是指针

  auto joint_name_left = info_.joints[0].name;
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_name_left, hardware_interface::HW_IF_VELOCITY, &left_command_speed_));

  auto joint_name_right = info_.joints[1].name;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_name_right, hardware_interface::HW_IF_VELOCITY, &right_command_speed_));

  return command_interfaces;
}

hardware_interface::CallbackReturn Tiger_motor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 在启动时command应该等于0
  left_command_speed_ = 0;
  right_command_speed_ = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tiger_motor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Tiger_motor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lg(mutex_);

  return hardware_interface::return_type::OK;
}

void Tiger_motor::SendSetPDOCmd(
  can_frame * tx_frame, unsigned short canID, unsigned short addr1, unsigned char subAddr,
  unsigned int data, unsigned char size)
{
  tx_frame->can_id = 0x0600 + canID;
  tx_frame->len = 8;
  switch (size) {
    case 1:
      tx_frame->data[0] = 0x2f;
      break;
    case 2:
      tx_frame->data[0] = 0x2b;
      break;
    case 3:
      tx_frame->data[0] = 0x27;
      break;
    case 4:
      tx_frame->data[0] = 0x23;
      break;
    default:
      return;
  }
  tx_frame->data[1] = addr1 & 0xff;
  tx_frame->data[2] = (addr1 >> 8) & 0xff;
  tx_frame->data[3] = subAddr;
  tx_frame->data[4] = data & 0xff;
  tx_frame->data[5] = (data >> 8) & 0xff;
  tx_frame->data[6] = (data >> 16) & 0xff;
  tx_frame->data[7] = (data >> 24) & 0xff;
  can_->write(tx_frame);
}

void Tiger_motor::start_up_NMT(can_frame * tx_frame, uint8_t canID)
{
  tx_frame->can_id = 0x00;
  tx_frame->len = 2;

  tx_frame->data[0] = 0x01;
  tx_frame->data[1] = canID;

  can_->write(tx_frame);
}

hardware_interface::return_type Tiger_motor::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  long long speed_left = left_command_speed_ * 10 / 6 * 4096 * 20;
  long long speed_right = right_command_speed_ * 10 / 6 * 4096 * 20;

  can_frame left_tx_frame;
  left_tx_frame.can_id = tx_id_left;
  left_tx_frame.len = 8;
  left_tx_frame.data[0] = CMD_SET_SPEED;
  left_tx_frame.data[1] = INDEX_SPEED % 256;
  left_tx_frame.data[2] = INDEX_SPEED / 256;
  left_tx_frame.data[3] = SUBINDEX_SPEED;
  left_tx_frame.data[4] = speed_left & 0X000000FF;
  left_tx_frame.data[5] = (speed_left >> 8) & 0X000000FF;
  left_tx_frame.data[6] = (speed_left >> 16) & 0X000000FF;
  left_tx_frame.data[7] = (speed_left >> 24) & 0X000000FF;

  can_frame right_tx_frame;
  right_tx_frame.can_id = tx_id_right;
  right_tx_frame.len = 8;
  right_tx_frame.data[0] = CMD_SET_SPEED;
  right_tx_frame.data[1] = INDEX_SPEED % 256;
  right_tx_frame.data[2] = INDEX_SPEED / 256;
  right_tx_frame.data[3] = SUBINDEX_SPEED;
  right_tx_frame.data[4] = speed_right & 0X000000FF;
  right_tx_frame.data[5] = (speed_right >> 8) & 0X000000FF;
  right_tx_frame.data[6] = (speed_right >> 16) & 0X000000FF;
  right_tx_frame.data[7] = (speed_right >> 24) & 0X000000FF;

  can_->write(&left_tx_frame);
  can_->write(&right_tx_frame);

  return hardware_interface::return_type::OK;
}

void Tiger_motor::callback(const can_frame & frame)
{
  if (!motor_inited_) {
    if (frame.can_id >= 0x0580 && frame.can_id <= 0x0585) {
      std::lock_guard<std::mutex> lg(mutex_);

      auto canid = frame.can_id - 0x0580;

      subIndex = frame.data[2];
      subIndex <<= 8;
      subIndex |= frame.data[1];

      Status = frame.data[5];
      Status <<= 8;
      Status |= frame.data[4];
      Status &= 0x01;

      if (mCurrentMotorStatusState == Status_INITING) {
        if ((mGetStatusMotorNo == canid) && (frame.data[0] == 0x4B /*默认4B*/) && (Status == 1)) {
          mGetStatusMotorStep += 1;
        }
      }

      //驱动器已就绪
      if (mCurrentMotorStatusState == Status_INITFINISH) {
        if (mCurrentMotorPDOState == PDO_INITING) {
          if (mPDOSetMotorNo == canid && frame.data[0] == 0x60) {
            mPDOSetMotorStep += 1;
          }
        }
      }
    }

    return;
  }

  // running
  if (frame.can_id >= 0x0280 && frame.can_id <= 0x0285) {
    std::lock_guard<std::mutex> lg(mutex_);

    auto canid = frame.can_id - 0x0280;
    if (canid == motor_id_left_) {
      left_state_position_mid = frame.data[3];
      left_state_position_mid = (left_state_position_mid << 8) + frame.data[2];
      left_state_position_mid = (left_state_position_mid << 8) + frame.data[1];
      left_state_position_mid = (left_state_position_mid << 8) + frame.data[0];
      left_state_position_ =
        static_cast<double>(left_state_position_mid) / 4096.0 / 20.0 * 2.0 * M_PI;
    } else if (canid == motor_id_right_) {
      right_state_position_mid = frame.data[3];
      right_state_position_mid = (right_state_position_mid << 8) + frame.data[2];
      right_state_position_mid = (right_state_position_mid << 8) + frame.data[1];
      right_state_position_mid = (right_state_position_mid << 8) + frame.data[0];
      right_state_position_ =
        static_cast<double>(right_state_position_mid) / 4096.0 / 20.0 * 2.0 * M_PI;
    }
  }
}

}  // namespace agv_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(agv_hw::Tiger_motor, hardware_interface::SystemInterface)