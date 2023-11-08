#include "humanoid_control/state_estimator.h"
#include "footlib.h"

namespace humanoid {

StateEstimator::StateEstimator(rclcpp::Node* node) : node_(node) {
    motor_feedback_sub_ =
        node_->create_subscription<humanoid_interface::msg::MotorFeedback>(
            "motor_feedback", rclcpp::SensorDataQoS(),
            std::bind(&StateEstimator::motor_feedback_callback_, this,
                      std::placeholders::_1));
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_state", 10);
    joint_state_pub_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&StateEstimator::publish_joint_state_, this));
}

void StateEstimator::motor_feedback_callback_(
    const humanoid_interface::msg::MotorFeedback::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(motor_feedback_data_mutex_);
    motor_feedback_data_[msg->id].position = msg->position;
    motor_feedback_data_[msg->id].velocity = msg->velocity;
    motor_feedback_data_[msg->id].torque = msg->torque;
}

void StateEstimator::publish_joint_state_() {
    std::lock_guard<std::mutex> lock(motor_feedback_data_mutex_);
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = node_->now();
    
    // joint1, joint2, joint3, joint4, joint5
    for (int i = 1; i < 6; ++i) {
        if (auto it = motor_feedback_data_.find(i); it != motor_feedback_data_.end()) {
            joint_state_msg.name.push_back("joint" + std::to_string(i));
            joint_state_msg.position.push_back(it->second.position);
            joint_state_msg.velocity.push_back(it->second.velocity);
            joint_state_msg.effort.push_back(it->second.torque);
        }
    }

    // joint6, joint7
    auto upper_motor = motor_feedback_data_.find(6);
    auto lower_motor = motor_feedback_data_.find(7);
    if (upper_motor != motor_feedback_data_.end() && lower_motor != motor_feedback_data_.end()) {
        Eigen::Matrix<double, 3, 2> upper_lower;
        Eigen::Matrix<double, 3, 2> roll_pitch;

        upper_lower << upper_motor->second.position, lower_motor->second.position,
                       upper_motor->second.velocity, lower_motor->second.velocity,
                       upper_motor->second.torque, lower_motor->second.torque;

        FootLib::forward(upper_lower, roll_pitch);

        joint_state_msg.name.push_back("joint6");
        joint_state_msg.position.push_back(roll_pitch(0, 0));
        joint_state_msg.velocity.push_back(roll_pitch(1, 0));
        joint_state_msg.effort.push_back(roll_pitch(2, 0));
        joint_state_msg.name.push_back("joint7");
        joint_state_msg.position.push_back(roll_pitch(0, 1));
        joint_state_msg.velocity.push_back(roll_pitch(1, 1));
        joint_state_msg.effort.push_back(roll_pitch(2, 1));
    }

    // joint8, joint9, joint10, joint11
    for (int i = 8; i < 12; ++i) {
        if (auto it = motor_feedback_data_.find(i); it != motor_feedback_data_.end()) {
            joint_state_msg.name.push_back("joint" + std::to_string(i));
            joint_state_msg.position.push_back(it->second.position);
            joint_state_msg.velocity.push_back(it->second.velocity);
            joint_state_msg.effort.push_back(it->second.torque);
        }
    }

    // joint12, joint13
    upper_motor = motor_feedback_data_.find(12);
    lower_motor = motor_feedback_data_.find(13);
    if (upper_motor != motor_feedback_data_.end() && lower_motor != motor_feedback_data_.end()) {
        Eigen::Matrix<double, 3, 2> upper_lower;
        Eigen::Matrix<double, 3, 2> roll_pitch;

        upper_lower << upper_motor->second.position, lower_motor->second.position,
                       upper_motor->second.velocity, lower_motor->second.velocity,
                       upper_motor->second.torque, lower_motor->second.torque;

        FootLib::forward(upper_lower, roll_pitch);

        joint_state_msg.name.push_back("joint12");
        joint_state_msg.position.push_back(roll_pitch(0, 0));
        joint_state_msg.velocity.push_back(roll_pitch(1, 0));
        joint_state_msg.effort.push_back(roll_pitch(2, 0));
        joint_state_msg.name.push_back("joint13");
        joint_state_msg.position.push_back(roll_pitch(0, 1));
        joint_state_msg.velocity.push_back(roll_pitch(1, 1));
        joint_state_msg.effort.push_back(roll_pitch(2, 1));
    }

    // joint14, joint15, ..., joint22
    for (int i = 14; i < 23; ++i) {
        if (auto it = motor_feedback_data_.find(i); it != motor_feedback_data_.end()) {
            joint_state_msg.name.push_back("joint" + std::to_string(i));
            joint_state_msg.position.push_back(it->second.position);
            joint_state_msg.velocity.push_back(it->second.velocity);
            joint_state_msg.effort.push_back(it->second.torque);
        }
    }

    joint_state_pub_->publish(joint_state_msg);
}

}  // namespace humanoid
