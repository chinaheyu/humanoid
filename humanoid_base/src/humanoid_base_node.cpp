#include "humanoid_base/humanoid_base_node.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "humanoid_base/rich.h"

namespace humanoid {

HumanoidBaseNode::HumanoidBaseNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("humanoid_base", options) {
    assert(protocol_is_supported());

    // Loading parameters add listing to parameter changing
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&HumanoidBaseNode::update_parameters_callback_, this,
                  std::placeholders::_1));
    load_parameters_();

    // Create ros publisher and subscription
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/raw", rclcpp::SensorDataQoS());
    motor_feedback_publisher_ =
        this->create_publisher<humanoid_interface::msg::MotorFeedback>(
            "motor_feedback", rclcpp::SensorDataQoS());
    motor_control_subscription_ =
        this->create_subscription<humanoid_interface::msg::MotorControl>(
            "motor_control", rclcpp::SystemDefaultsQoS(),
            std::bind(&HumanoidBaseNode::motor_control_callback_, this,
                      std::placeholders::_1));
    face_control_subscription_ =
        this->create_subscription<humanoid_interface::msg::FaceControl>(
            "face_control", rclcpp::SystemDefaultsQoS(),
            std::bind(&HumanoidBaseNode::face_control_callback_, this,
                      std::placeholders::_1));
    neck_control_subscription_ =
        this->create_subscription<humanoid_interface::msg::NeckControl>(
            "neck_control", rclcpp::SystemDefaultsQoS(),
            std::bind(&HumanoidBaseNode::neck_control_callback_, this,
                      std::placeholders::_1));
    head_feedback_publisher_ =
        this->create_publisher<humanoid_interface::msg::HeadFeedback>(
            "head_feedback", rclcpp::SensorDataQoS());

    // Create timer for auto scan usb device
    scan_device_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&HumanoidBaseNode::scan_device_, this));

    // Do first scan
    scan_device_();
}

void HumanoidBaseNode::dispatch_frame_(
    const std::shared_ptr<const SerialManager>& from, uint16_t cmd_id,
    const uint8_t* data, size_t len) {
    // Reveived feedback from serial device.
    switch (cmd_id) {
        case CMD_GYRO_FEEDBACK:
            if (len == sizeof(cmd_gyro_feedback_t)) {
                publish_imu_(
                    "s" + from->get_serial_info().serial_number,
                    ZeroCopyMessage<uint8_t, cmd_gyro_feedback_t>(data));
            }
            break;
        case CMD_MOTOR_FEEDBACK:
            if (len == sizeof(cmd_motor_feedback_t)) {
                publish_motor_feedback_(
                    ZeroCopyMessage<uint8_t, cmd_motor_feedback_t>(data));
            }
            break;
        case CMD_HEAD_FEEDBACK:
            if (len == sizeof(cmd_head_feedback_t)) {
                publish_head_feedback_(
                    ZeroCopyMessage<uint8_t, cmd_head_feedback_t>(data));
            }
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
    for (const auto& info : LibUSBWarpper::get_instance().list_device()) {
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

void HumanoidBaseNode::publish_imu_(
    const std::string& frame_id,
    const ZeroCopyMessage<uint8_t, cmd_gyro_feedback_t>& data) {
    tf2::Quaternion q;
    q.setRPY(data.get_message().roll / 1000.0,
             data.get_message().pitch / 1000.0,
             data.get_message().yaw / 1000.0);

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp =
        rclcpp::Time(data.get_message().timestamp * 1000, RCL_SYSTEM_TIME);
    msg.orientation = tf2::toMsg(q);
    imu_publisher_->publish(msg);
}

void HumanoidBaseNode::publish_motor_feedback_(
    const ZeroCopyMessage<uint8_t, cmd_motor_feedback_t>& data) {
    auto it = params_.motors.find(data.get_message().id);
    if (it != params_.motors.end()) {
        // angle transform
        float motor_position =
            data.get_message().position / 1000.0 - it->second.offset;
        if (it->second.reverse) {
            motor_position = -motor_position;
        }

        // normize angle
        motor_position = fmodf(motor_position, 2 * M_PI);
        if (motor_position > M_PI) {
            motor_position -= 2 * M_PI;
        }
        if (motor_position < -M_PI) {
            motor_position += 2 * M_PI;
        }

        humanoid_interface::msg::MotorFeedback msg;
        msg.stamp =
            rclcpp::Time(data.get_message().timestamp * 1000, RCL_SYSTEM_TIME);
        msg.id = data.get_message().id;
        msg.position = motor_position;
        msg.velocity = data.get_message().velocity / 1000.0;
        msg.torque = data.get_message().torque / 1000.0;
        motor_feedback_publisher_->publish(msg);
    }
}

void HumanoidBaseNode::publish_head_feedback_(
    const ZeroCopyMessage<uint8_t, cmd_head_feedback_t>& data) {
    humanoid_interface::msg::HeadFeedback msg;
    msg.stamp =
        rclcpp::Time(data.get_message().timestamp * 1000, RCL_SYSTEM_TIME);
    for (size_t i = 0; i < msg.pulse_width.size(); i++) {
        msg.pulse_width[i] = data.get_message().pulse_width[i];
    }
    msg.pitch_velocity = data.get_message().pitch_velocity / 1000.0;
    msg.yaw_angle = data.get_message().yaw_angle / 1000.0;
    head_feedback_publisher_->publish(msg);
}

void HumanoidBaseNode::motor_control_callback_(
    const humanoid_interface::msg::MotorControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    std::unordered_map<uint8_t, Parameters::Motor>::iterator it =
        params_.motors.find(msg->id);
    if (it != params_.motors.end()) {
        auto serial_ptr = serials_.find(it->second.device);
        if (serial_ptr == serials_.end()) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Serial device does not exist: " << it->second.device);
        } else {
            float target_position =
                (it->second.reverse ? -msg->position : msg->position) +
                it->second.offset;
            if (msg->control_type ==
                humanoid_interface::msg::MotorControl::MOTOR_MIT_CONTROL) {
                cmd_motor_mit_t mit_msg;
                mit_msg.id = msg->id;
                mit_msg.position = target_position * 1000;
                mit_msg.velocity = msg->velocity * 1000;
                mit_msg.torque = msg->torque * 1000;
                mit_msg.kp = msg->kp * 1000;
                mit_msg.kd = msg->kp * 1000;
                serial_ptr->second->send_message_to_device(CMD_MOTOR_MIT,
                                                           mit_msg);
            }
            if (msg->control_type ==
                humanoid_interface::msg::MotorControl::MOTOR_POSITION_CONTROL) {
                cmd_motor_position_t position_msg;
                position_msg.id = msg->id;
                position_msg.position = target_position * 1000;
                serial_ptr->second->send_message_to_device(CMD_MOTOR_POSITION,
                                                           position_msg);
            }
        }
    }
}

void HumanoidBaseNode::face_control_callback_(
    const humanoid_interface::msg::FaceControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    auto serial_ptr = serials_.find(params_.head_device);
    if (serial_ptr == serials_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial device does not exist: "
                                                    << params_.head_device);
    } else {
        cmd_head_servo_t cmd;
        for (size_t i = 0; i < msg->pulse_width.size(); ++i) {
            cmd.pulse_width[i] = msg->pulse_width[i];
        }
        serial_ptr->second->send_message_to_device(CMD_HEAD_SERVO, cmd);
    }
}

void HumanoidBaseNode::neck_control_callback_(
    const humanoid_interface::msg::NeckControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    auto serial_ptr = serials_.find(params_.head_device);
    if (serial_ptr == serials_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial device does not exist: "
                                                    << params_.head_device);
    } else {
        cmd_neck_motor_t cmd;
        cmd.pitch_velocity = msg->pitch_velocity * 1000;
        cmd.yaw_angle = msg->yaw_angle * 1000;
        cmd.yaw_max_velocity = msg->yaw_max_velocity * 1000;
        serial_ptr->second->send_message_to_device(CMD_NECK_MOTOR, cmd);
    }
}

void HumanoidBaseNode::load_parameters_() {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);

    // Load head parameters
    this->get_parameter_or<std::string>("head", params_.head_device, "unknown");
    RCLCPP_INFO_STREAM(this->get_logger(), CL_BOLDGREEN << "Load head: "
                                                        << "head"
                                                        << " -> "
                                                        << params_.head_device
                                                        << CL_RESET);

    // Load motor parameters
    for (const auto& param_name : this->list_parameters({"motors"}, 3).names) {
        uint8_t motor_id;
        std::string key;
        if (parse_motor_parameter_(param_name, motor_id, key)) {
            if (key == "device") {
                params_.motors[motor_id].device =
                    this->get_parameter(param_name).as_string();
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   CL_BOLDGREEN
                                       << "Set motor " << key << ": "
                                       << params_.motors[motor_id].device
                                       << CL_RESET);
            }
            if (key == "reverse") {
                params_.motors[motor_id].reverse =
                    this->get_parameter(param_name).as_bool();
                RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    CL_BOLDGREEN
                        << "Set motor " << key << ": " << std::boolalpha
                        << params_.motors[motor_id].reverse << CL_RESET);
            }
            if (key == "offset") {
                params_.motors[motor_id].offset =
                    this->get_parameter(param_name).as_double();
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   CL_BOLDGREEN
                                       << "Set motor " << key << ": "
                                       << params_.motors[motor_id].offset
                                       << CL_RESET);
            }
        }
    }
}

bool HumanoidBaseNode::parse_motor_parameter_(const std::string& param_name,
                                              uint8_t& id_out,
                                              std::string& key_out) {
    if (param_name.find("motor_", 7) != std::string::npos) {
        id_out = static_cast<uint8_t>(std::stoi(
            param_name.substr(13, param_name.find_last_of(".") - 13)));
        if (params_.motors.count(id_out) == 0) {
            Parameters::Motor motor;
            motor.device = "unknown";
            motor.id = id_out;
            motor.reverse = false;
            motor.offset = 0.0;
            params_.motors.emplace(id_out, std::move(motor));
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN << "New motor: motor_"
                                            << unsigned(id_out) << CL_RESET);
        }
        key_out = param_name.substr(param_name.find_last_of(".") + 1);
        return true;
    }
    return false;
}

void HumanoidBaseNode::update_parameters_callback_(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);

    for (const auto& param : msg->new_parameters) {
        RCLCPP_INFO_STREAM(this->get_logger(), "New parameter: " << param.name);
        if (param.name == "head") {
            params_.head_device = param.value.string_value;
        }
        if (param.name.find("motors.motor_") != std::string::npos) {
            uint8_t motor_id;
            std::string key;
            if (parse_motor_parameter_(param.name, motor_id, key)) {
                if (key == "device") {
                    params_.motors[motor_id].device = param.value.string_value;
                }
                if (key == "reverse") {
                    params_.motors[motor_id].reverse = param.value.bool_value;
                }
                if (key == "offset") {
                    params_.motors[motor_id].offset = param.value.double_value;
                }
            }
        }
    }
    for (const auto& param : msg->deleted_parameters) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Delete parameter: " << param.name);
        if (param.name == "head") {
            params_.head_device = "unknown";
        }
        if (param.name.find("motors.motor_") != std::string::npos) {
            uint8_t motor_id;
            std::string key;
            if (parse_motor_parameter_(param.name, motor_id, key)) {
                if (key == "device") {
                    params_.motors[motor_id].device = "unknown";
                }
                if (key == "reverse") {
                    params_.motors[motor_id].reverse = false;
                }
                if (key == "offset") {
                    params_.motors[motor_id].offset = 0.0;
                }
            }
        }
    }
    for (const auto& param : msg->changed_parameters) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Change parameter: " << param.name);
        if (param.name == "head") {
            params_.head_device = param.value.string_value;
        }
        if (param.name.find("motors.motor_") != std::string::npos) {
            uint8_t motor_id;
            std::string key;
            if (parse_motor_parameter_(param.name, motor_id, key)) {
                if (key == "device") {
                    params_.motors[motor_id].device = param.value.string_value;
                }
                if (key == "reverse") {
                    params_.motors[motor_id].reverse = param.value.bool_value;
                }
                if (key == "offset") {
                    params_.motors[motor_id].offset = param.value.double_value;
                }
            }
        }
    }
}

}  // namespace humanoid

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(humanoid::HumanoidBaseNode)