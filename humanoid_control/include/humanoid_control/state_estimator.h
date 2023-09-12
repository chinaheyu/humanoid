#ifndef __STATE_ESTIMATOR_H__
#define __STATE_ESTIMATOR_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <humanoid_interface/msg/motor_feedback.hpp>
#include <unordered_map>
#include <mutex>

namespace humanoid {

struct MotorFeedbackData {
    double position;
    double velocity;
    double torque;
};

class StateEstimator {
public:
    explicit StateEstimator(rclcpp::Node* node);

private:
    rclcpp::Node* node_;
    rclcpp::Subscription<humanoid_interface::msg::MotorFeedback>::SharedPtr motor_feedback_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::ConstSharedPtr joint_state_pub_timer_;

    std::mutex motor_feedback_data_mutex_;
    std::unordered_map<uint8_t, MotorFeedbackData> motor_feedback_data_;

    void motor_feedback_callback_(const humanoid_interface::msg::MotorFeedback::SharedPtr msg);

    void publish_joint_state_();

};

}  // namespace humanoid

#endif  // __STATE_ESTIMATOR_H__
