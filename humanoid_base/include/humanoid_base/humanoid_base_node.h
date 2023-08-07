#ifndef __HUMANOID_BASE_NODE__
#define __HUMANOID_BASE_NODE__

#include <memory>
#include <mutex>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <unordered_map>

#include "humanoid_base/serial_manager.h"
#include "humanoid_interface/msg/face_control.hpp"
#include "humanoid_interface/msg/motor_control.hpp"
#include "humanoid_interface/msg/motor_feedback.hpp"
#include "humanoid_interface/msg/neck_control.hpp"
#include "humanoid_interface/msg/head_feedback.hpp"

class HumanoidBaseNode : public rclcpp::Node {
public:
    HumanoidBaseNode();

private:
    std::mutex serials_container_mutex_;
    std::unordered_map<std::string, std::shared_ptr<SerialManager>> serials_;
    std::unordered_map<uint8_t, std::string> motor_mapping_;
    std::string head_serial_;
    rclcpp::TimerBase::ConstSharedPtr scan_device_timer_;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<humanoid_interface::msg::MotorControl>::SharedPtr
        motor_control_subscription_;
    rclcpp::Publisher<humanoid_interface::msg::MotorFeedback>::SharedPtr
        motor_feedback_publisher_;
    rclcpp::Subscription<humanoid_interface::msg::FaceControl>::SharedPtr
        face_control_subscription_;
    rclcpp::Subscription<humanoid_interface::msg::NeckControl>::SharedPtr
        neck_control_subscription_;
    rclcpp::Publisher<humanoid_interface::msg::HeadFeedback>::SharedPtr
        head_feedback_publisher_;

    void dispatch_frame_(const std::shared_ptr<const SerialManager>& from,
                         uint16_t cmd_id,
                         std::shared_ptr<std::vector<uint8_t>>&& data);

    template <typename T1, typename T2>
    inline const T1* get_message_(T2 data);

    void load_parameters_();

    void update_parameters_callback_(
        const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);

    void scan_device_();

    void publish_imu_(const std::string& frame_id, long long timestamp,
                      float roll, float pitch, float yaw);
    
    void publish_head_feedback_(std::shared_ptr<std::vector<uint8_t>>& data);

    void publish_motor_feedback_(long long timestamp, uint8_t id,
                                 float position, float velocity, float torque);

    void motor_control_callback_(
        const humanoid_interface::msg::MotorControl::SharedPtr msg);

    void face_control_callback_(
        const humanoid_interface::msg::FaceControl::SharedPtr msg);

    void neck_control_callback_(
        const humanoid_interface::msg::NeckControl::SharedPtr msg);

    friend void* read_from_serial_port_(void* obj);
};

#endif  // __HUMANOID_BASE_NODE__
