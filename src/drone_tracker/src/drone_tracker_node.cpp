#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <string>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using vision_msgs::msg::Detection2DArray;

static float wrap_pi(float x)
{
  while (x > static_cast<float>(M_PI)) x -= static_cast<float>(2.0 * M_PI);
  while (x < -static_cast<float>(M_PI)) x += static_cast<float>(2.0 * M_PI);
  return x;
}

class DroneTrackerNode : public rclcpp::Node
{
public:
  DroneTrackerNode() : Node("drone_tracker_node")
  {
    this->declare_parameter<int>("vehicle_id", 2);  // 控制的无人机系统编号（PX4 MAV_SYS_ID / target_system）
    this->declare_parameter<std::string>("detections_topic", "/drone_detections");  // 目标检测话题（vision_msgs/Detection2DArray）
    this->declare_parameter<std::string>("target_class", "");  // 目标类别名过滤（空字符串表示不过滤）
    this->declare_parameter<int>("image_width", 640);  // 图像宽度（像素），用于把cx偏差归一化
    this->declare_parameter<double>("hfov_deg", 90.0);  // 相机水平视场角（度），用于像素偏差->角度误差近似换算
    this->declare_parameter<double>("kp", 0.2);  // yaw跟随比例系数（越大转得越快/越激进）
    this->declare_parameter<double>("sign", -1.0);  // 方向因子（发现朝向反了就改成 -1.0 / 1.0）
    this->declare_parameter<int>("center_deadband_px", 8);  // 画面中心死区（像素），小于该值不调yaw以减少抖动
    this->declare_parameter<double>("error_lowpass_alpha", 0.6);  // yaw误差低通滤波系数(0~1)，越大越跟得紧但越抖
    this->declare_parameter<double>("center_weight", 0.6);  // 多目标时选目标的“靠近中心”权重，越大越不跳目标
    this->declare_parameter<double>("min_score", 0.25);  // 最小置信度，低于该值的检测忽略
    this->declare_parameter<double>("hover_x", 3.0);  // 悬停位置X（本地/NED坐标，单位m）
    this->declare_parameter<double>("hover_y", 0.0);  // 悬停位置Y（单位m）
    this->declare_parameter<double>("hover_z", -5.0);  // 悬停位置Z（NED向下为负，单位m）
    this->declare_parameter<double>("scan_rate_rad_s", 0.5);  // 搜索模式自转角速度（rad/s）
    this->declare_parameter<double>("lost_timeout_s", 0.35);  // 多久没检测到就判定丢失并恢复搜索（秒）
    this->declare_parameter<double>("max_yaw_step_rad", 0.25);  // 每个控制周期最大yaw步长（rad），防止过冲/过快
    this->declare_parameter<int>("control_period_ms", 100);  // 控制循环周期（ms）

    vehicle_id_ = this->get_parameter("vehicle_id").as_int();
    if (vehicle_id_ < 1) {
      RCLCPP_WARN(this->get_logger(), "vehicle_id=%d 非法，已强制改为 1", vehicle_id_);
      vehicle_id_ = 1;
    }

    // px4 multi-vehicle topic prefix:
    // - vehicle_id==1 -> "/fmu/..."
    // - vehicle_id>=2 -> "/px4_{vehicle_id-1}/fmu/..."
    if (vehicle_id_ == 1) {
      px4_prefix_ = "";
    } else {
      px4_prefix_ = "/px4_" + std::to_string(vehicle_id_ - 1);
    }

    detections_topic_ = this->get_parameter("detections_topic").as_string();
    target_class_ = this->get_parameter("target_class").as_string();
    image_width_ = this->get_parameter("image_width").as_int();
    hfov_rad_ = static_cast<float>(this->get_parameter("hfov_deg").as_double() * M_PI / 180.0);
    kp_ = static_cast<float>(this->get_parameter("kp").as_double());
    sign_ = static_cast<float>(this->get_parameter("sign").as_double());
    center_deadband_px_ = this->get_parameter("center_deadband_px").as_int();
    error_alpha_ = static_cast<float>(this->get_parameter("error_lowpass_alpha").as_double());
    center_weight_ = static_cast<float>(this->get_parameter("center_weight").as_double());
    min_score_ = static_cast<float>(this->get_parameter("min_score").as_double());
    hover_x_ = static_cast<float>(this->get_parameter("hover_x").as_double());
    hover_y_ = static_cast<float>(this->get_parameter("hover_y").as_double());
    hover_z_ = static_cast<float>(this->get_parameter("hover_z").as_double());
    scan_rate_ = static_cast<float>(this->get_parameter("scan_rate_rad_s").as_double());
    lost_timeout_s_ = this->get_parameter("lost_timeout_s").as_double();
    max_step_ = static_cast<float>(this->get_parameter("max_yaw_step_rad").as_double());
    control_period_ms_ = this->get_parameter("control_period_ms").as_int();
    pub_offboard_ = this->create_publisher<OffboardControlMode>(px4_prefix_ + "/fmu/in/offboard_control_mode", 10);
    pub_traj_ = this->create_publisher<TrajectorySetpoint>(px4_prefix_ + "/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<VehicleCommand>(px4_prefix_ + "/fmu/in/vehicle_command", 10);

    sub_det_ = this->create_subscription<Detection2DArray>(
      detections_topic_, 10,
      [this](const Detection2DArray::SharedPtr msg) { this->onDetections(*msg); });

    offboard_counter_ = 0;
    yaw_sp_ = 0.0f;
    last_det_time_ = this->get_clock()->now();
    last_yaw_err_ = 0.0f;
    filt_yaw_err_ = 0.0f;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      [this]() { this->onTimer(); });

    RCLCPP_INFO(
      this->get_logger(),
      "drone_tracker_node: vehicle_id=%d px4_prefix=%s detections=%s hover=(%.2f,%.2f,%.2f) period=%dms deadband_px=%d alpha=%.2f",
      vehicle_id_, px4_prefix_.c_str(), detections_topic_.c_str(), hover_x_, hover_y_, hover_z_, control_period_ms_,
      center_deadband_px_, error_alpha_);
  }

private:
  void onDetections(const Detection2DArray & msg)
  {
    if (image_width_ <= 0) return;

    const float half_w = 0.5f * static_cast<float>(image_width_);
    const float center_x = half_w;

    // 选择目标：兼顾置信度与“靠近画面中心”的稳定性，避免多目标/抖动导致频繁跳目标
    float best_utility = -1e9f;
    float best_cx = 0.0f;
    bool found = false;

    for (const auto & det : msg.detections) {
      if (det.results.empty()) continue;
      const auto & hyp = det.results[0].hypothesis;
      const std::string cls = hyp.class_id;
      const float score = static_cast<float>(hyp.score);
      if (!target_class_.empty() && cls != target_class_) continue;
      if (score < min_score_) continue;

      const float cx = static_cast<float>(det.bbox.center.position.x);
      float norm = (cx - center_x) / half_w;  // [-1,1]
      norm = std::max(-1.0f, std::min(1.0f, norm));
      const float utility = score - center_weight_ * std::fabs(norm);

      if (utility > best_utility) {
        best_utility = utility;
        best_cx = static_cast<float>(det.bbox.center.position.x);
        found = true;
      }
    }
    if (!found) return;

    last_det_time_ = this->get_clock()->now();
    det_fresh_.store(true, std::memory_order_relaxed);

    float norm = (best_cx - center_x) / half_w;  // [-1,1]
    norm = std::max(-1.0f, std::min(1.0f, norm));

    // 死区：避免“已经在中心附近”还在不停微调导致抖动
    if (center_deadband_px_ > 0 && std::fabs(best_cx - center_x) <= static_cast<float>(center_deadband_px_)) {
      last_yaw_err_ = 0.0f;
    } else {
      last_yaw_err_ = norm * (0.5f * hfov_rad_);
    }

    // 对误差做低通滤波，降低检测抖动导致的yaw来回摆
    const float a = std::max(0.0f, std::min(1.0f, error_alpha_));
    filt_yaw_err_ = a * last_yaw_err_ + (1.0f - a) * filt_yaw_err_;
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

  void setOffboard() { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); }
  void arm() { publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f); }

  void onTimer()
  {
    // offboard control mode needs to be paired with trajectory setpoint
    OffboardControlMode ocm{};
    ocm.position = true;
    ocm.velocity = false;
    ocm.acceleration = false;
    ocm.attitude = false;
    ocm.body_rate = false;
    ocm.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    pub_offboard_->publish(ocm);

    if (offboard_counter_ == 10) {
      setOffboard();
      arm();
    }
    if (offboard_counter_ < 11) {
      offboard_counter_++;
    }

    const auto now = this->get_clock()->now();
    const double age_s = (now - last_det_time_).nanoseconds() * 1e-9;
    const float dt = static_cast<float>(control_period_ms_) / 1000.0f;

    // 一旦识别到目标，立刻停止旋转并进入跟随
    const bool fresh = det_fresh_.exchange(false, std::memory_order_relaxed);
    const bool tracking = fresh || (age_s < lost_timeout_s_);

    if (tracking) {
      if (fresh) {
        filt_yaw_err_ = last_yaw_err_;
      }
      float step = sign_ * kp_ * filt_yaw_err_;
      step = std::max(-max_step_, std::min(max_step_, step));
      yaw_sp_ = wrap_pi(yaw_sp_ + step);
    } else {
      yaw_sp_ = wrap_pi(yaw_sp_ + scan_rate_ * dt);
    }

    TrajectorySetpoint sp{};
    sp.position = {hover_x_, hover_y_, hover_z_};
    sp.yaw = yaw_sp_;
    sp.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    pub_traj_->publish(sp);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Subscription<Detection2DArray>::SharedPtr sub_det_;

  int vehicle_id_{2};
  std::string px4_prefix_;

  std::string detections_topic_;
  std::string target_class_;
  int image_width_{640};
  float hfov_rad_{static_cast<float>(M_PI / 2.0)};
  float kp_{1.0f};
  float sign_{1.0f};
  int center_deadband_px_{8};
  float error_alpha_{0.6f};
  float center_weight_{0.6f};
  float min_score_{0.25f};
  float hover_x_{3.0f};
  float hover_y_{0.0f};
  float hover_z_{-5.0f};
  float scan_rate_{0.8f};
  double lost_timeout_s_{0.35};
  float max_step_{0.25f};
  int control_period_ms_{100};

  uint64_t offboard_counter_{0};
  float yaw_sp_{0.0f};
  rclcpp::Time last_det_time_;
  float last_yaw_err_{0.0f};
  float filt_yaw_err_{0.0f};
  std::atomic<bool> det_fresh_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

