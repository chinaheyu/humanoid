#include "humanoid_base/humanoid_base_node.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "humanoid_base/protocol_def.h"
#include "humanoid_base/rich.h"

namespace humanoid {

HumanoidBaseNode::HumanoidBaseNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("humanoid_base", options), head_serial_{"unknown"} {
    assert(protocol_is_supported());

    // Loading parameters add listing to parameter changing
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&HumanoidBaseNode::update_parameters_callback_, this,
                  std::placeholders::_1));
    load_parameters_();

    // Create ros publisher and subscription
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
    std::shared_ptr<std::vector<uint8_t>>&& data) {
    // Reveived feedback from serial device.
    switch (cmd_id) {
        case CMD_GYRO_FEEDBACK:
            publish_imu_("s" + from->get_serial_info().serial_number, data);
            break;
        case CMD_MOTOR_FEEDBACK:
            publish_motor_feedback_(data);
            break;
        case CMD_HEAD_FEEDBACK:
            publish_head_feedback_(data);
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
    const std::string& frame_id, std::shared_ptr<std::vector<uint8_t>>& data) {
    tf2::Quaternion q;
    q.setRPY(get_message_<cmd_gyro_feedback_t>(data)->roll / 1000.0,
             get_message_<cmd_gyro_feedback_t>(data)->pitch / 1000.0,
             get_message_<cmd_gyro_feedback_t>(data)->yaw / 1000.0);

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp =
        rclcpp::Time(get_message_<cmd_gyro_feedback_t>(data)->timestamp * 1000,
                     RCL_STEADY_TIME);
    msg.orientation = tf2::toMsg(q);
    imu_publisher_->publish(msg);
}

void HumanoidBaseNode::publish_motor_feedback_(
    std::shared_ptr<std::vector<uint8_t>>& data) {
    humanoid_interface::msg::MotorFeedback msg;
    msg.stamp =
        rclcpp::Time(get_message_<cmd_motor_feedback_t>(data)->timestamp * 1000,
                     RCL_STEADY_TIME);
    msg.id = get_message_<cmd_motor_feedback_t>(data)->id;
    msg.position = get_message_<cmd_motor_feedback_t>(data)->position / 1000.0;
    msg.velocity = get_message_<cmd_motor_feedback_t>(data)->velocity / 1000.0;
    msg.torque = get_message_<cmd_motor_feedback_t>(data)->torque / 1000.0;
    motor_feedback_publisher_->publish(msg);
}

void HumanoidBaseNode::publish_head_feedback_(
    std::shared_ptr<std::vector<uint8_t>>& data) {
    humanoid_interface::msg::HeadFeedback msg;
    msg.stamp =
        rclcpp::Time(get_message_<cmd_head_feedback_t>(data)->timestamp * 1000,
                     RCL_STEADY_TIME);
    for (size_t i = 0; i < msg.pulse_width.size(); i++) {
        msg.pulse_width[i] =
            get_message_<cmd_head_feedback_t>(data)->pulse_width[i];
    }
    msg.pitch_velocity =
        get_message_<cmd_head_feedback_t>(data)->pitch_velocity / 1000.0;
    msg.yaw_angle = get_message_<cmd_head_feedback_t>(data)->yaw_angle / 1000.0;
    head_feedback_publisher_->publish(msg);
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
        auto serial_ptr = serials_.find(it->second);
        if (serial_ptr == serials_.end()) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Serial device does not exist: " << it->second);
        } else {
            if (msg->control_type ==
                humanoid_interface::msg::MotorControl::MOTOR_MIT_CONTROL) {
                cmd_motor_mit_t mit_msg;
                mit_msg.id = msg->id;
                mit_msg.position = msg->position * 1000;
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
                position_msg.position = msg->position * 1000;
                serial_ptr->second->send_message_to_device(CMD_MOTOR_POSITION,
                                                           position_msg);
            }
        }
    }
}

void HumanoidBaseNode::face_control_callback_(
    const humanoid_interface::msg::FaceControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);
    auto serial_ptr = serials_.find(head_serial_);
    if (serial_ptr == serials_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Serial device does not exist: " << head_serial_);
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
    auto serial_ptr = serials_.find(head_serial_);
    if (serial_ptr == serials_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Serial device does not exist: " << head_serial_);
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
    for (const auto& param_name : this->list_parameters({}, 1).names) {
        // Load motor parameters
        if (param_name.rfind("motor_", 0) == 0) {
            uint8_t motor_id =
                static_cast<uint8_t>(std::stoi(param_name.substr(6)));
            motor_mapping_[motor_id] =
                this->get_parameter(param_name).as_string();
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                CL_BOLDGREEN << "Load motor mapping: " << param_name << " -> "
                             << motor_mapping_[motor_id] << CL_RESET);
        }

        // Load head parameters
        if (param_name.compare("head") == 0) {
            head_serial_ = this->get_parameter(param_name).as_string();
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN
                                   << "Load head mapping: " << param_name
                                   << " -> " << head_serial_ << CL_RESET);
        }
    }
}

void HumanoidBaseNode::update_parameters_callback_(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(serials_container_mutex_);

    // Update parameters
    for (const auto& param : msg->new_parameters) {
        if (param.value.type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            // Update motor parameter
            if (param.name.rfind("motor_", 0) == 0) {
                uint8_t motor_id =
                    static_cast<uint8_t>(std::stoi(param.name.substr(6)));
                motor_mapping_[motor_id] = param.value.string_value;
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   CL_BOLDGREEN
                                       << "New motor mapping: " << param.name
                                       << " -> " << param.value.string_value
                                       << CL_RESET);
            }
            // Update head parameter
            if (param.name.compare("head") == 0) {
                head_serial_ = param.value.string_value;
                RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    CL_BOLDGREEN << "New head mapping: " << param.name << " -> "
                                 << param.value.string_value << CL_RESET);
            }
        }
    }
    for (const auto& param : msg->deleted_parameters) {
        // Update motor parameter
        if (param.name.rfind("motor_", 0) == 0) {
            uint8_t motor_id =
                static_cast<uint8_t>(std::stoi(param.name.substr(6)));
            motor_mapping_.erase(motor_id);
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN << "Delete motor mapping: "
                                            << param.name << CL_RESET);
        }
        // Update head parameter
        if (param.name.compare("head") == 0) {
            head_serial_ = "unknown";
            RCLCPP_INFO_STREAM(this->get_logger(),
                               CL_BOLDGREEN << "Delete head mapping: "
                                            << param.name << CL_RESET);
        }
    }
    for (const auto& param : msg->changed_parameters) {
        if (param.value.type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            // Update motor parameter
            if (param.name.rfind("motor_", 0) == 0) {
                uint8_t motor_id =
                    static_cast<uint8_t>(std::stoi(param.name.substr(6)));
                std::unordered_map<uint8_t, std::string>::iterator it =
                    motor_mapping_.find(motor_id);
                if (it != motor_mapping_.end()) {
                    it->second = param.value.string_value;
                    RCLCPP_INFO_STREAM(
                        this->get_logger(),
                        CL_BOLDGREEN << "Change motor mapping: " << param.name
                                     << " -> " << param.value.string_value
                                     << CL_RESET);
                }
            }
            // Update head parameter
            if (param.name.compare("head") == 0) {
                head_serial_ = param.value.string_value;
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   CL_BOLDGREEN
                                       << "Change head mapping: " << param.name
                                       << " -> " << param.value.string_value
                                       << CL_RESET);
            }
        }
    }
}
}  // namespace humanoid

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(humanoid::HumanoidBaseNode)