#include "humanoid_base/humanoid_base_node.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "humanoid_base/protocol_def.h"
#include "humanoid_base/rich.h"

HumanoidBaseNode::HumanoidBaseNode()
    : rclcpp::Node("humanoid_base",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {
    assert(protocol_is_supported());

    load_motor_mapping_parameters_();

    scan_device_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&HumanoidBaseNode::scan_device_, this));
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS());
    motor_feedback_publisher_ =
        this->create_publisher<humanoid_interface::msg::MotorFeedback>(
            "motor_feedback", rclcpp::SensorDataQoS());
    motor_control_subscription_ =
        this->create_subscription<humanoid_interface::msg::MotorControl>(
            "motor_control", rclcpp::SystemDefaultsQoS(),
            std::bind(&HumanoidBaseNode::motor_control_callback_, this,
                      std::placeholders::_1));

    // Initialize
    scan_device_();
}

void HumanoidBaseNode::dispatch_frame_(
    const std::shared_ptr<const SerialManager>& from, uint16_t cmd_id,
    std::shared_ptr<std::vector<uint8_t>>&& data) {
    // Reveived feedback from serial device.
    switch (cmd_id) {
        case CMD_GYRO_FEEDBACK:
            publish_imu_(
                "s" + from->get_serial_info().serial_number,
                get_message_<cmd_gyro_feedback_t>(data)->timestamp,
                get_message_<cmd_gyro_feedback_t>(data)->roll / 1000.0,
                get_message_<cmd_gyro_feedback_t>(data)->pitch / 1000.0,
                get_message_<cmd_gyro_feedback_t>(data)->yaw / 1000.0);
            break;
        case CMD_MOTOR_FEEDBACK:
            publish_motor_feedback_(
                get_message_<cmd_motor_feedback_t>(data)->timestamp,
                get_message_<cmd_motor_feedback_t>(data)->id,
                get_message_<cmd_motor_feedback_t>(data)->position,
                get_message_<cmd_motor_feedback_t>(data)->velocity,
                get_message_<cmd_motor_feedback_t>(data)->torque);
            break;
        default:
            break;
    }
}

void HumanoidBaseNode::scan_device_() {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);

    // Remove offline serial device.
    for (auto it = serials_.begin(); it != serials_.end();) {
        if (it->second->is_wait_to_delete()) {
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Detach humanoid serial device: "
                                   << it->second->get_serial_info());
            it = serials_.erase(it);
        } else
            ++it;
    }

    // Scan serial device.
    for (const auto& info : serial::SerialInfo::list_port()) {
        if (info.manufacturer == "scut" && info.product == "humanoid") {
            if (serials_.count(info.serial_number) == 0) {
                RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    CL_BOLDGREEN << "Found humanoid serial device: " << info
                                 << CL_RESET);
                serials_[info.serial_number] =
                    std::make_shared<SerialManager>(info, this);
            }
        }
    }
}

void HumanoidBaseNode::publish_imu_(const std::string& frame_id,
                                    long long timestamp, float roll,
                                    float pitch, float yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Time(timestamp * 1000, RCL_STEADY_TIME);
    msg.orientation = tf2::toMsg(q);
    imu_publisher_->publish(msg);
}

void HumanoidBaseNode::publish_motor_feedback_(long long timestamp, uint8_t id,
                                               float position, float velocity,
                                               float torque) {
    humanoid_interface::msg::MotorFeedback msg;
    msg.stamp = rclcpp::Time(timestamp * 1000, RCL_STEADY_TIME);
    msg.id = id;
    msg.position = position;
    msg.velocity = velocity;
    msg.torque = torque;
    motor_feedback_publisher_->publish(msg);
}

template <typename T1, typename T2>
inline const T1* HumanoidBaseNode::get_message_(T2 data) {
    assert(data->size() == sizeof(T1));
    return reinterpret_cast<const T1*>(&(*data)[0]);
}

void HumanoidBaseNode::motor_control_callback_(
    const humanoid_interface::msg::MotorControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    std::unordered_map<uint8_t, std::string>::iterator it =
        motor_mapping_.find(msg->id);
    if (it != motor_mapping_.end()) {
        if (msg->control_type ==
            humanoid_interface::msg::MotorControl::MOTOR_MIT_CONTROL) {
            cmd_motor_mit_t mit_msg;
            mit_msg.id = msg->id;
            mit_msg.position = msg->position * 1000;
            mit_msg.velocity = msg->velocity * 1000;
            mit_msg.torque = msg->torque * 1000;
            mit_msg.kp = msg->kp * 1000;
            mit_msg.kd = msg->kp * 1000;
            serials_[it->second]->send_message_to_device(CMD_MOTOR_MIT,
                                                         mit_msg);
        }
        if (msg->control_type ==
            humanoid_interface::msg::MotorControl::MOTOR_POSITION_CONTROL) {
            cmd_motor_position_t position_msg;
            position_msg.id = msg->id;
            position_msg.position = msg->position * 1000;
            serials_[it->second]->send_message_to_device(CMD_MOTOR_POSITION,
                                                         position_msg);
        }
    }
}

void HumanoidBaseNode::load_motor_mapping_parameters_() {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    for (const auto& param : this->list_parameters({}, 1).names) {
        if (param.rfind("motor_", 0) == 0) {
            uint8_t motor_id = static_cast<uint8_t>(std::stoi(param.substr(6)));
            motor_mapping_[motor_id] = param;
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                CL_BOLDGREEN << "Load motor mapping: " << param << CL_RESET);
        }
    }
}

void HumanoidBaseNode::update_parameters_callback_(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);

    // Update motor mapping
    for (const auto& param : msg->new_parameters) {
        if (param.value.type ==
                rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
            param.name.rfind("motor_", 0) == 0) {
            uint8_t motor_id =
                static_cast<uint8_t>(std::stoi(param.name.substr(6)));
            motor_mapping_[motor_id] = param.value.string_value;
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN << "New motor mapping: "
                                            << param.name << CL_RESET);
        }
    }
    for (const auto& param : msg->deleted_parameters) {
        if (param.value.type ==
                rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
            param.name.rfind("motor_", 0) == 0) {
            uint8_t motor_id =
                static_cast<uint8_t>(std::stoi(param.name.substr(6)));
            motor_mapping_.erase(motor_id);
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN << "Delete motor mapping: "
                                            << param.name << CL_RESET);
        }
    }
    for (const auto& param : msg->changed_parameters) {
        if (param.value.type ==
                rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
            param.name.rfind("motor_", 0) == 0) {
            uint8_t motor_id =
                static_cast<uint8_t>(std::stoi(param.name.substr(6)));
            std::unordered_map<uint8_t, std::string>::iterator it =
                motor_mapping_.find(motor_id);
            if (it != motor_mapping_.end()) {
                it->second = param.value.string_value;
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   CL_BOLDGREEN << "Change motor mapping: "
                                                << param.name << CL_RESET);
            }
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanoidBaseNode>());
    rclcpp::shutdown();
    return 0;
}
