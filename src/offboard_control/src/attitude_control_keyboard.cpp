#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
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
using px4_msgs::msg::VehicleLocalPosition;

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

// 简单定高控制参数（在本节点内部代替 px4ctrl 做一个高度环）
static constexpr float ALT_STEP_FINE      = 0.1f;      // 高度微调（米）
static constexpr float ALT_STEP_COARSE    = 0.5f;      // 高度粗调（米）
static constexpr float ALT_KP             = 0.4f;      // 高度 P 增益
static constexpr float ALT_KD             = 0.1f;      // 高度 D 增益（对 z 轴速度）
static constexpr float ALT_KI             = 0.05f;     // 高度 I 增益（对 z 轴误差积分，建议很小）
static constexpr float ALT_I_LIMIT        = 0.4f;      // 积分项限幅（对应推力修正量上限）

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

    // 订阅 PX4 本身估计的本地位置，用于简单定高控制（z 为 NED 下轴，向下为正）
    sub_local_pos_ = this->create_subscription<VehicleLocalPosition>(
      px4_prefix_ + "/fmu/out/vehicle_local_position",
      rclcpp::SensorDataQoS().best_effort(),   // 匹配 PX4 默认 QoS，避免 QoS 不兼容
      [this](const VehicleLocalPosition::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        alt_z_ = msg->z;
        alt_vz_ = msg->vz;
        have_alt_.store(true, std::memory_order_relaxed);
      });

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
      "        yaw:   T/G 微调(%.1f°)  Y/H 粗调(%.1f°)\n"
      "        高度: X 开关定高模式  I/K 微调(%.1fm)  O/L 粗调(%.1fm)\n"
      "        Q 退出\n",
      STEP_THRUST_FINE, STEP_THRUST_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE,
      STEP_ANGLE_FINE, STEP_ANGLE_COARSE,
      ALT_STEP_FINE, ALT_STEP_COARSE);
  }

  void printParams()
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    const char *mode_str = altitude_hold_enabled_ ? "ALT_HOLD" : "MANUAL  ";
    printf("\033[A\r  mode=%s  thrust_cmd=%.4f  hover_thrust=%.4f"
           "  roll=%.2f  pitch=%.2f  yaw=%.2f (deg)"
           "  z=%.2f  z_sp=%.2f   ",
      mode_str,
      last_thrust_cmd_, hover_thrust_,
      roll_deg_, pitch_deg_, yaw_deg_,
      alt_z_, alt_sp_);
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
          // 一键定高模式开关与目标高度调节（基于 PX4 VehicleLocalPosition.z，向下为正）
          case 'x': case 'X':
            if (!altitude_hold_enabled_) {
              if (have_alt_.load(std::memory_order_relaxed)) {
                altitude_hold_enabled_ = true;
                alt_sp_ = alt_z_;
                alt_i_ = 0.0f;
                updated = true;
                RCLCPP_INFO(this->get_logger(),
                  "Altitude hold ENABLED, lock current z=%.2f m (NED down), use I/K O/L 调高度",
                  alt_z_);
              } else {
                RCLCPP_WARN(this->get_logger(),
                  "Altitude hold: no VehicleLocalPosition yet, cannot enable.");
              }
            } else {
              altitude_hold_enabled_ = false;
              alt_i_ = 0.0f;
              updated = true;
              RCLCPP_INFO(this->get_logger(), "Altitude hold DISABLED, back to manual thrust.");
            }
            break;
          case 'i': case 'I':
            if (altitude_hold_enabled_ && have_alt_.load(std::memory_order_relaxed)) {
              alt_sp_ -= ALT_STEP_FINE;   // 减小 z => 上升
              updated = true;
            }
            break;
          case 'k': case 'K':
            if (altitude_hold_enabled_ && have_alt_.load(std::memory_order_relaxed)) {
              alt_sp_ += ALT_STEP_FINE;   // 增大 z => 下降
              updated = true;
            }
            break;
          case 'o': case 'O':
            if (altitude_hold_enabled_ && have_alt_.load(std::memory_order_relaxed)) {
              alt_sp_ -= ALT_STEP_COARSE;
              updated = true;
            }
            break;
          case 'l': case 'L':
            if (altitude_hold_enabled_ && have_alt_.load(std::memory_order_relaxed)) {
              alt_sp_ += ALT_STEP_COARSE;
              updated = true;
            }
            break;
          case 'q': case 'Q': quit_.store(true); rclcpp::shutdown(); break;
          default: break;
        }
      }
      if (updated) {
        printParams();   // 仅在有按键导致参数/模式变化时刷新一行状态
      }
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

    float roll_rad, pitch_rad, yaw_rad;
    float thrust;
    bool alt_hold = false;
    float z = 0.0f, vz = 0.0f, z_sp = 0.0f;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      roll_rad = roll_deg_ * static_cast<float>(M_PI / 180.0);
      pitch_rad = pitch_deg_ * static_cast<float>(M_PI / 180.0);
      yaw_rad = yaw_deg_ * static_cast<float>(M_PI / 180.0);
      thrust = hover_thrust_;
      alt_hold = altitude_hold_enabled_;
      z = alt_z_;
      vz = alt_vz_;
      z_sp = alt_sp_;
    }

    // 简单定高：使用 PX4 的 VehicleLocalPosition.z / vz 做一个 PID 高度环（带积分限幅），调节 thrust
    if (alt_hold && have_alt_.load(std::memory_order_relaxed)) {
      const float err = z - z_sp;   // NED 下轴：z 大说明更“低”，需要更大推力
      const float dt = std::max(1e-3f, static_cast<float>(control_period_ms_) / 1000.0f);

      float i_term = 0.0f;
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        alt_i_ = std::clamp(alt_i_ + err * dt, -ALT_I_LIMIT / std::max(1e-6f, ALT_KI), ALT_I_LIMIT / std::max(1e-6f, ALT_KI));
        i_term = ALT_KI * alt_i_;
      }

      const float u = ALT_KP * err + ALT_KD * vz + i_term;
      const float thrust_unsat = thrust + u;
      thrust = std::clamp(thrust_unsat, 0.0f, 1.0f);

      // 简单 anti-windup：如果推力饱和且误差还在推动更饱和方向，就回退本次积分
      if ((thrust != thrust_unsat) && ((thrust == 1.0f && err > 0.0f) || (thrust == 0.0f && err < 0.0f))) {
        std::lock_guard<std::mutex> lock(params_mutex_);
        alt_i_ = std::clamp(alt_i_ - err * dt, -ALT_I_LIMIT / std::max(1e-6f, ALT_KI), ALT_I_LIMIT / std::max(1e-6f, ALT_KI));
      }
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

    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      last_thrust_cmd_ = thrust;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr pub_attitude_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_cmd_;

  // PX4 本地位置订阅（用于定高控制，z/vz）
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr sub_local_pos_;

  int vehicle_id_{1};
  std::string px4_prefix_;
  std::mutex params_mutex_;
  float hover_thrust_{0.729f};
  float roll_deg_{0.0f};
  float pitch_deg_{0.0f};
  float yaw_deg_{0.0f};
  int control_period_ms_{20};
  uint64_t offboard_counter_{0};

  // 简单定高控制相关状态
  bool altitude_hold_enabled_{false};
  float alt_z_{0.0f};     // 当前 z（NED，下为正）
  float alt_vz_{0.0f};    // 当前 z 方向速度
  float alt_sp_{0.0f};    // 目标 z（与 VehicleLocalPosition.z 同符号）
  std::atomic<bool> have_alt_{false};
  float alt_i_{0.0f};     // 高度误差积分状态（内部状态）
  float last_thrust_cmd_{0.0f};  // 上一次发送的推力（便于终端显示）

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
