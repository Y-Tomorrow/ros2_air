#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>

#if defined(__linux__)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#endif

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;

/** 欧拉角 (roll,pitch,yaw) 转四元数 [qw,qx,qy,qz]，NED 到 body，ZYX 顺序。并归一化。 */
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
  float n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n > 1e-6f) { qw /= n; qx /= n; qy /= n; qz /= n; }
}

// 键盘控制步长：敏感参数用更小增量
static constexpr float STEP_THRUST_FINE   = 0.0002f;   // 推力微调
static constexpr float STEP_THRUST_COARSE = 0.002f;    // 推力粗调
static constexpr float STEP_ANGLE_FINE    = 0.2f;      // 姿态角微调（度）
static constexpr float STEP_ANGLE_COARSE  = 1.0f;      // 姿态角粗调（度）

class AttitudeControlKeyboardNode : public rclcpp::Node
{
public:
  AttitudeControlKeyboardNode() : rclcpp::Node("attitude_control_keyboard")
  {
    this->declare_parameter<int>("vehicle_id", 1);
    this->declare_parameter<double>("hover_thrust", 0.729f);
    this->declare_parameter<double>("roll_deg", 0.0);
    this->declare_parameter<double>("pitch_deg", 0.0);
    this->declare_parameter<double>("yaw_deg", 0.0);
    this->declare_parameter<int>("control_period_ms", 20);

    vehicle_id_ = std::max(1, static_cast<int>(this->get_parameter("vehicle_id").as_int()));
    px4_prefix_ = (vehicle_id_ == 1) ? "" : ("/px4_" + std::to_string(vehicle_id_ - 1));

    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      hover_thrust_ = static_cast<float>(this->get_parameter("hover_thrust").as_double());
      roll_deg_ = static_cast<float>(this->get_parameter("roll_deg").as_double());
      pitch_deg_ = static_cast<float>(this->get_parameter("pitch_deg").as_double());
      yaw_deg_ = static_cast<float>(this->get_parameter("yaw_deg").as_double());
    }
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

    printHelp();
    keyboard_thread_ = std::thread(&AttitudeControlKeyboardNode::keyboardLoop, this);
  }

  ~AttitudeControlKeyboardNode()
  {
    quit_.store(true);
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
#if defined(__linux__)
    if (termios_ok_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &term_orig_);
      if (stdin_flags_orig_ >= 0)
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags_orig_);
    }
#endif
  }

private:
  void printHelp()
  {
    RCLCPP_INFO(this->get_logger(),
      "\n  [键盘] 推力: W/S 微调(%.4f)  U/J 粗调(%.3f)\n"
      "        roll:  E/D 微调(%.1f°)  C/V 粗调(%.1f°)\n"
      "        pitch: R/F 微调(%.1f°)  B/N 粗调(%.1f°)\n"
      "        yaw:   T/G 微调(%.1f°)  Y/H 粗调(%.1f°)   Q 退出\n",
      STEP_THRUST_FINE, STEP_THRUST_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE);
  }

  void printParams()
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    printf("\r  hover_thrust=%.4f  roll=%.2f  pitch=%.2f  yaw=%.2f (deg)   ",
      hover_thrust_, roll_deg_, pitch_deg_, yaw_deg_);
    fflush(stdout);
  }

  void keyboardLoop()
  {
#if defined(__linux__)
    termios_ok_ = (tcgetattr(STDIN_FILENO, &term_orig_) == 0);
    if (termios_ok_) {
      struct termios t = term_orig_;
      t.c_lflag &= ~(ICANON | ECHO);
      t.c_cc[VMIN] = 0;
      t.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &t);
      stdin_flags_orig_ = fcntl(STDIN_FILENO, F_GETFL, 0);
      if (stdin_flags_orig_ >= 0)
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags_orig_ | O_NONBLOCK);
    }
#endif
    while (!quit_.load()) {
#if defined(__linux__)
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      struct timeval tv = { 0, 200000 };
      if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
#endif
      int c = getchar();
      if (c == EOF) continue;

      bool updated = false;
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        switch (c) {
          case 'w': case 'W': hover_thrust_ = std::min(1.0f, hover_thrust_ + STEP_THRUST_FINE); updated = true; break;
          case 's': case 'S': hover_thrust_ = std::max(0.0f, hover_thrust_ - STEP_THRUST_FINE); updated = true; break;
          case 'u': case 'U': hover_thrust_ = std::min(1.0f, hover_thrust_ + STEP_THRUST_COARSE); updated = true; break;
          case 'j': case 'J': hover_thrust_ = std::max(0.0f, hover_thrust_ - STEP_THRUST_COARSE); updated = true; break;
          case 'e': case 'E': roll_deg_ += STEP_ANGLE_FINE; updated = true; break;
          case 'd': case 'D': roll_deg_ -= STEP_ANGLE_FINE; updated = true; break;
          case 'c': case 'C': roll_deg_ += STEP_ANGLE_COARSE; updated = true; break;
          case 'v': case 'V': roll_deg_ -= STEP_ANGLE_COARSE; updated = true; break;
          case 'r': case 'R': pitch_deg_ += STEP_ANGLE_FINE; updated = true; break;
          case 'f': case 'F': pitch_deg_ -= STEP_ANGLE_FINE; updated = true; break;
          case 'b': case 'B': pitch_deg_ += STEP_ANGLE_COARSE; updated = true; break;
          case 'n': case 'N': pitch_deg_ -= STEP_ANGLE_COARSE; updated = true; break;
          case 't': case 'T': yaw_deg_ += STEP_ANGLE_FINE; updated = true; break;
          case 'g': case 'G': yaw_deg_ -= STEP_ANGLE_FINE; updated = true; break;
          case 'y': case 'Y': yaw_deg_ += STEP_ANGLE_COARSE; updated = true; break;
          case 'h': case 'H': yaw_deg_ -= STEP_ANGLE_COARSE; updated = true; break;
          case 'q': case 'Q': quit_.store(true); rclcpp::shutdown(); break;
          default: break;
        }
      }
      if (updated) printParams();
    }
  }

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
    if (offboard_counter_ < 11) offboard_counter_++;

    float roll_rad, pitch_rad, yaw_rad, thrust;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      roll_rad = roll_deg_ * static_cast<float>(M_PI / 180.0);
      pitch_rad = pitch_deg_ * static_cast<float>(M_PI / 180.0);
      yaw_rad = yaw_deg_ * static_cast<float>(M_PI / 180.0);
      thrust = hover_thrust_;
    }

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
    sp.thrust_body[2] = -thrust;
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
  std::mutex params_mutex_;
  float hover_thrust_{0.729f};
  float roll_deg_{0.0f};
  float pitch_deg_{0.0f};
  float yaw_deg_{0.0f};
  int control_period_ms_{20};
  uint64_t offboard_counter_{0};

  std::thread keyboard_thread_;
  std::atomic<bool> quit_{false};
#if defined(__linux__)
  struct termios term_orig_{};
  bool termios_ok_{false};
  int stdin_flags_orig_{-1};
#endif
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudeControlKeyboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
