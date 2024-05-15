#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <can_msgs/msg/frame.hpp>
#include <cmath>

class AutowareToCantx : public rclcpp::Node
{
public:
  AutowareToCantx() : Node("autoware_to_cantx")
  {
    // Autowareからの入力トピックのサブスクライバーの初期化
    control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 10,
        std::bind(&AutowareToCantx::controlCmdCallback, this, std::placeholders::_1));

    turn_indicators_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", 10,
        std::bind(&AutowareToCantx::turnIndicatorsCmdCallback, this, std::placeholders::_1));

    actuation_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
        "/control/command/actuation_cmd", 10,
        std::bind(&AutowareToCantx::actuationCmdCallback, this, std::placeholders::_1));

    // CANトピックのパブリッシャーの初期化
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/can_tx", 10);
  }

private:
  // 制御コマンドのコールバック関数
  void controlCmdCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received control command.");

    // CANフレームを作成（ステアリング角度用）
    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = this->get_clock()->now();
    can_msg.id = 0x7DF;
    can_msg.is_rtr = false;
    can_msg.is_extended = false;
    can_msg.is_error = false;
    can_msg.dlc = 8;

    // ステアリング角度のデータをCANフレームに設定
    // ラジアンから16ビット整数への変換（±45度 = ±0.785ラジアンを最大値としてスケーリング）
    int16_t steering_angle = static_cast<int16_t>(msg->lateral.steering_tire_angle * (32767 / 0.785));

    can_msg.data[0] = 0x08; // size
    can_msg.data[1] = 0x08; // mode
    can_msg.data[2] = 0x21; // PID for steering angle
    can_msg.data[3] = (steering_angle >> 8) & 0xFF; // High byte of steering angle
    can_msg.data[4] = steering_angle & 0xFF; // Low byte of steering angle

    // 残りのデータバイトをゼロで埋める
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    // CANメッセージを送信
    can_pub_->publish(can_msg);
  }

  // ターンインジケータコマンドのコールバック関数
  void turnIndicatorsCmdCallback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received turn indicators command.");

    // CANフレームを作成（ターンインジケータ用）
    can_msgs::msg::Frame can_msg;
    can_msg.header.stamp = this->get_clock()->now();
    can_msg.id = 0x7DF;
    can_msg.is_rtr = true;
    can_msg.is_extended = false;
    can_msg.is_error = false;
    can_msg.dlc = 8;

    // ターンインジケータのデータをCANフレームに設定
    uint8_t turn_signal;
    switch (msg->command) {
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND:
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE:
        turn_signal = 0; // OFF
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
        turn_signal = 2; // Turn L
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
        turn_signal = 1; // Turn R
        break;
      default:
        turn_signal = 0; // デフォルトはOFF
        break;
    }

    can_msg.data[0] = 0x08; // size
    can_msg.data[1] = 0x08; // mode
    can_msg.data[2] = 0x37; // PID for turn signal
    can_msg.data[3] = turn_signal; // Turn signal command

    // 残りのデータバイトをゼロで埋める
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    // CANメッセージを送信
    can_pub_->publish(can_msg);
  }

  // アクチュエーションコマンドのコールバック関数
  void actuationCmdCallback(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received actuation command.");

    // CANフレームを作成（スロットル開度用）
    can_msgs::msg::Frame can_msg_throttle;
    can_msg_throttle.header.stamp = this->get_clock()->now();
    can_msg_throttle.id = 0x7DF;
    can_msg_throttle.is_rtr = true;
    can_msg_throttle.is_extended = false;
    can_msg_throttle.is_error = false;
    can_msg_throttle.dlc = 8;

    // スロットル開度のデータをCANフレームに設定
    int16_t throttle_opening = static_cast<int16_t>(msg->actuation.accel_cmd * 32767);
    can_msg_throttle.data[0] = 0x08; // size
    can_msg_throttle.data[1] = 0x08; // mode
    can_msg_throttle.data[2] = 0x34; // PID for throttle opening
    can_msg_throttle.data[3] = (throttle_opening >> 8) & 0xFF; // High byte of throttle opening
    can_msg_throttle.data[4] = throttle_opening & 0xFF; // Low byte of throttle opening

    // 残りのデータバイトをゼロで埋める
    can_msg_throttle.data[5] = 0x00;
    can_msg_throttle.data[6] = 0x00;
    can_msg_throttle.data[7] = 0x00;

    // CANメッセージを送信（スロットル開度）
    can_pub_->publish(can_msg_throttle);

    // CANフレームを作成（ブレーキ力用）
    can_msgs::msg::Frame can_msg_brake;
    can_msg_brake.header.stamp = this->get_clock()->now();
    can_msg_brake.id = 0x7DF;
    can_msg_brake.is_rtr = true;
    can_msg_brake.is_extended = false;
    can_msg_brake.is_error = false;
    can_msg_brake.dlc = 8;

    // ブレーキ力のデータをCANフレームに設定
    int16_t brake_force = static_cast<int16_t>(msg->actuation.brake_cmd * 32767);
    can_msg_brake.data[0] = 0x08; // size
    can_msg_brake.data[1] = 0x08; // mode
    can_msg_brake.data[2] = 0x35; // PID for brake force
    can_msg_brake.data[3] = (brake_force >> 8) & 0xFF; // High byte of brake force
    can_msg_brake.data[4] = brake_force & 0xFF; // Low byte of brake force

    // 残りのデータバイトをゼロで埋める
    can_msg_brake.data[5] = 0x00;
    can_msg_brake.data[6] = 0x00;
    can_msg_brake.data[7] = 0x00;

    // CANメッセージを送信（ブレーキ力）
    can_pub_->publish(can_msg_brake);
  }

  // サブスクライバー
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub_;

  // パブリッシャー
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareToCantx>());
  rclcpp::shutdown();
  return 0;
}
