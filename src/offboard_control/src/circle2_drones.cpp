#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DualCircleControl : public rclcpp::Node
{
public:
    DualCircleControl() : Node("dual_circle_control")
    {
        // 第一个无人机的发布者 (/fmu/...)
        offboard_control_mode_publisher_1_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_1_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_1_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // 第二个无人机的发布者 (/px4_1/fmu/...)
        offboard_control_mode_publisher_2_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_2_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_2_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_1_ = 0;
        offboard_setpoint_counter_2_ = 0;
        time_step_ = 0;

        auto timer_callback = [this]() -> void {
            // 第一个无人机的控制逻辑
            if (offboard_setpoint_counter_1_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(1, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm(1);
            }

            // 第二个无人机的控制逻辑
            if (offboard_setpoint_counter_2_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(2, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm(2);
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode(1);
            publish_offboard_control_mode(2);
            publish_trajectory_setpoint(1);
            publish_trajectory_setpoint(2);

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_1_ < 11) {
                offboard_setpoint_counter_1_++;
            }
            if (offboard_setpoint_counter_2_ < 11) {
                offboard_setpoint_counter_2_++;
            }

            time_step_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm(int drone_id);
    void disarm(int drone_id);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    // 第一个无人机的发布者
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_1_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_1_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_1_;

    // 第二个无人机的发布者
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_2_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_2_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_2_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_1_;   //!< counter for the number of setpoints sent for drone 1
    uint64_t offboard_setpoint_counter_2_;   //!< counter for the number of setpoints sent for drone 2
    uint64_t time_step_;

    void publish_offboard_control_mode(int drone_id);
    void publish_trajectory_setpoint(int drone_id);
    void publish_vehicle_command(int drone_id, uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 * @param drone_id 无人机ID (1 或 2)
 */
void DualCircleControl::arm(int drone_id)
{
    publish_vehicle_command(drone_id, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send to drone %d", drone_id);
}

/**
 * @brief Send a command to Disarm the vehicle
 * @param drone_id 无人机ID (1 或 2)
 */
void DualCircleControl::disarm(int drone_id)
{
    publish_vehicle_command(drone_id, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send to drone %d", drone_id);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 * @param drone_id 无人机ID (1 或 2)
 */
void DualCircleControl::publish_offboard_control_mode(int drone_id)
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    if (drone_id == 1) {
        offboard_control_mode_publisher_1_->publish(msg);
    } else {
        offboard_control_mode_publisher_2_->publish(msg);
    }
}

/**
 * @brief Publish a trajectory setpoint for circular trajectory
 * @param drone_id 无人机ID (1 或 2)
 */
void DualCircleControl::publish_trajectory_setpoint(int drone_id)
{
    TrajectorySetpoint msg{};
    double t = static_cast<double>(time_step_) * 0.1; // time in seconds
    
    double x, y, z, yaw;
    
    if (drone_id == 1) {
        // 第一个无人机：半径为2.0的圆形轨迹
        double radius = 2.0;
        double omega = 0.5;
        double center_x = 0.0;
        double center_y = 0.0;
        
        x = center_x + radius * std::cos(omega * t);
        y = center_y + radius * std::sin(omega * t);
        z = -5.0; // altitude
        yaw = omega * t + M_PI/2; // [-PI:PI]
    } else {
        // 第二个无人机：半径为1.5的圆形轨迹，中心偏移
        double radius = 1.5;
        double omega = 0.5;
        double center_x = 3.0;  // 偏移中心位置
        double center_y = 0.0;
        
        x = center_x + radius * std::cos(omega * t);
        y = center_y + radius * std::sin(omega * t);
        z = -5.0; // altitude
        // yaw = omega * t + M_PI/2; // [-PI:PI]
    }

    msg.position = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z)
    };
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    if (drone_id == 1) {
        trajectory_setpoint_publisher_1_->publish(msg);
    } else {
        trajectory_setpoint_publisher_2_->publish(msg);
    }
}

/**
 * @brief Publish vehicle commands
 * @param drone_id 无人机ID (1 或 2)
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void DualCircleControl::publish_vehicle_command(int drone_id, uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = drone_id;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    if (drone_id == 1) {
        vehicle_command_publisher_1_->publish(msg);
    } else {
        vehicle_command_publisher_2_->publish(msg);
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting dual circle control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualCircleControl>());

    rclcpp::shutdown();
    return 0;
}
