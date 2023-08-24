#ifndef __HUMANOID_BASE_NODE__
#define __HUMANOID_BASE_NODE__

#include <memory>
#include <mutex>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <unordered_map>

#include "humanoid_base/protocol_def.h"
#include "humanoid_base/serial_manager.h"
#include "humanoid_base/span.h"
#include "humanoid_interface/msg/face_control.hpp"
#include "humanoid_interface/msg/head_feedback.hpp"
#include "humanoid_interface/msg/motor_control.hpp"
#include "humanoid_interface/msg/motor_feedback.hpp"
#include "humanoid_interface/msg/neck_control.hpp"
#include "humanoid_interface/visibility_control.h"

namespace humanoid {

class HumanoidBaseNode : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    explicit HumanoidBaseNode(const rclcpp::NodeOptions& options);

private:
    struct Parameters {
        struct Motor {
            std::string device;
            uint8_t id;
            bool reverse;
            double offset;
            std::string mapping;
        };
        std::unordered_map<uint8_t, Motor> motors;
        std::string head_device;
    } params_;
    std::mutex serials_container_mutex_;
    std::unordered_map<std::string, std::shared_ptr<SerialManager>> serials_;
    rclcpp::TimerBase::ConstSharedPtr scan_device_timer_;
    rclcpp::TimerBase::ConstSharedPtr publish_joint_state_timer_;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
        parameter_event_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<humanoid_interface::msg::MotorControl>::SharedPtr
        motor_control_subscription_;
    rclcpp::Publisher<humanoid_interface::msg::MotorFeedback>::SharedPtr
        motor_feedback_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;
    rclcpp::Subscription<humanoid_interface::msg::FaceControl>::SharedPtr
        face_control_subscription_;
    rclcpp::Subscription<humanoid_interface::msg::NeckControl>::SharedPtr
        neck_control_subscription_;
    rclcpp::Publisher<humanoid_interface::msg::HeadFeedback>::SharedPtr
        head_feedback_publisher_;
    std::unordered_map<std::string, double> joint_state_;
    std::unordered_map<std::string, double> foot_state_;

    void dispatch_frame_(const std::shared_ptr<const SerialManager>& from,
                         uint16_t cmd_id, const uint8_t* data, size_t len);

    void load_parameters_();

    bool parse_motor_parameter_(const std::string& param_name, uint8_t& id_out,
                                std::string& key_out);

    void update_parameters_callback_(
        const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);

    void scan_device_();

    void publish_joint_state_();

    void publish_imu_(
        const std::string& frame_id,
        const ZeroCopyMessage<uint8_t, cmd_gyro_feedback_t>& data);

    void publish_head_feedback_(
        const ZeroCopyMessage<uint8_t, cmd_head_feedback_t>& data);

    void publish_motor_feedback_(
        const ZeroCopyMessage<uint8_t, cmd_motor_feedback_t>& data);

    void motor_control_callback_(
        const humanoid_interface::msg::MotorControl::SharedPtr msg);

    void face_control_callback_(
        const humanoid_interface::msg::FaceControl::SharedPtr msg);

    void neck_control_callback_(
        const humanoid_interface::msg::NeckControl::SharedPtr msg);

    friend class SerialManager;
};

}  // namespace humanoid

#endif  // __HUMANOID_BASE_NODE__
