#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <cmath>
#include <cstdint>
#include <chrono>

using namespace std::chrono_literals;

/** 兼容 launch 传入整数：先按 double 取，失败则按 int 取再转 double */
static double get_param_double(const rclcpp::Node & node, const std::string & name)
{
  try {
    return node.get_parameter(name).as_double();
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    return static_cast<double>(node.get_parameter(name).as_int());
  }
}
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;

/** 欧拉角 (roll, pitch, yaw) 转四元数 q = [qw, qx, qy, qz]，用于 VehicleAttitudeSetpoint.q_d */
static void euler_to_quat(float roll, float pitch, float yaw,
  float & qw, float & qx, float & qy, float & qz)
{
  float cr = std::cos(roll * 0.5f);
  float sr = std::sin(roll * 0.5f);
  float cp = std::cos(pitch * 0.5f);
  float sp = std::sin(pitch * 0.5f);
  float cy = std::cos(yaw * 0.5f);
  float sy = std::sin(yaw * 0.5f);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

class AttitudeControlNode : public rclcpp::Node
{
public:
  AttitudeControlNode() : rclcpp::Node("attitude_control")
  {
    this->declare_parameter<int>("vehicle_id", 1);
    this->declare_parameter<double>("hover_thrust", 0.6);   // 悬停推力 [0,1]，多旋翼 NED 下一般为负的归一化推力，这里用正值表示“向上”
    this->declare_parameter<double>("roll_deg", 0.0);         // 基础 roll 角（度）
    this->declare_parameter<double>("pitch_deg", 0.0);         // 基础 pitch 角（度）
    this->declare_parameter<double>("yaw_deg", 0.0);          // 基础 yaw 角（度）
    this->declare_parameter<int>("control_period_ms", 20);    // 控制周期 20ms -> 50Hz

    vehicle_id_ = this->get_parameter("vehicle_id").as_int();
    if (vehicle_id_ < 1) vehicle_id_ = 1;

    if (vehicle_id_ == 1) {
      px4_prefix_ = "";
    } else {
      px4_prefix_ = "/px4_" + std::to_string(vehicle_id_ - 1);
    }

    hover_thrust_ = static_cast<float>(get_param_double(*this, "hover_thrust"));
    roll_deg_ = static_cast<float>(get_param_double(*this, "roll_deg"));
    pitch_deg_ = static_cast<float>(get_param_double(*this, "pitch_deg"));
    yaw_deg_ = static_cast<float>(get_param_double(*this, "yaw_deg"));
    control_period_ms_ = this->get_parameter("control_period_ms").as_int();

    pub_offboard_ = this->create_publisher<OffboardControlMode>(
      px4_prefix_ + "/fmu/in/offboard_control_mode", 10);
    pub_attitude_ = this->create_publisher<VehicleAttitudeSetpoint>(
      px4_prefix_ + "/fmu/in/vehicle_attitude_setpoint", 10);
    pub_cmd_ = this->create_publisher<VehicleCommand>(
      px4_prefix_ + "/fmu/in/vehicle_command", 10);

    offboard_counter_ = 0;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      [this]() { this->onTimer(); });

    RCLCPP_INFO(this->get_logger(),
      "attitude_control: vehicle_id=%d prefix=%s hover_thrust=%.2f roll=%.1f pitch=%.1f yaw=%.1f period=%dms",
      vehicle_id_, px4_prefix_.c_str(), hover_thrust_, roll_deg_, pitch_deg_, yaw_deg_, control_period_ms_);
  }

private:
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(vehicle_id_);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    pub_cmd_->publish(msg);
  }

  void onTimer()
  {
    const uint64_t t_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);

    OffboardControlMode ocm{};
    ocm.position = false;
    ocm.velocity = false;
    ocm.acceleration = false;
    ocm.attitude = true;
    ocm.body_rate = false;
    ocm.timestamp = t_us;
    pub_offboard_->publish(ocm);

    if (offboard_counter_ == 10) {
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      RCLCPP_INFO(this->get_logger(), "Offboard + Arm sent");
    }
    if (offboard_counter_ < 11) {
      offboard_counter_++;
    }

    float roll_rad = roll_deg_ * static_cast<float>(M_PI / 180.0);
    float pitch_rad = pitch_deg_ * static_cast<float>(M_PI / 180.0);
    float yaw_rad = yaw_deg_ * static_cast<float>(M_PI / 180.0);

    float qw, qx, qy, qz;
    euler_to_quat(roll_rad, pitch_rad, yaw_rad, qw, qx, qy, qz);

    VehicleAttitudeSetpoint sp{};
    sp.timestamp = t_us;
    sp.roll_body = roll_rad;
    sp.pitch_body = pitch_rad;
    sp.yaw_body = yaw_rad;
    sp.yaw_sp_move_rate = 0.0f;
    sp.q_d[0] = qw;
    sp.q_d[1] = qx;
    sp.q_d[2] = qy;
    sp.q_d[3] = qz;
    sp.thrust_body[0] = 0.0f;
    sp.thrust_body[1] = 0.0f;
    sp.thrust_body[2] = -hover_thrust_;
    sp.reset_integral = false;
    sp.fw_control_yaw_wheel = false;
    pub_attitude_->publish(sp);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr pub_attitude_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_cmd_;

  int vehicle_id_{1};
  std::string px4_prefix_;
  float hover_thrust_{0.45f};
  float roll_deg_{0.0f};
  float pitch_deg_{0.0f};
  float yaw_deg_{0.0f};
  int control_period_ms_{20};

  uint64_t offboard_counter_{0};
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttitudeControlNode>());
  rclcpp::shutdown();
  return 0;
}
