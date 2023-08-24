#include "humanoid_base/serial_manager.h"

#include <alloca.h>

#include <chrono>

#include "humanoid_base/humanoid_base_node.h"
#include "humanoid_base/protocol_def.h"
#include "humanoid_base/rich.h"

namespace humanoid {

SerialManager::SerialManager(const USBDeviceInfo& i,
                             HumanoidBaseNode* const node,
                             long long sync_duration, long long read_timeout)
    : USBDevice(i),
      is_open_(false),
      is_sync_(false),
      wait_to_delete_(false),
      node_(node),
      sync_duration_(sync_duration),
      read_timeout_(read_timeout),
      last_sync_time_(0),
      app_id_(0),
      read_buffer_(4096, 0) {
    latency_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "latency/s" + info_.serial_number, 10);
    unpack_stream_obj_ = protocol_create_unpack_stream(1000, true);
    read_last_time_ = get_timestamp_();
    create_communication_thread_();
}

SerialManager::~SerialManager() {
    is_open_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    protocol_free_unpack_stream(unpack_stream_obj_);
}

void SerialManager::sync_time_() {
    long long current_time = get_timestamp_();
    RCLCPP_DEBUG_STREAM(
        node_->get_logger(),
        "Sync device time (timestamp = " << current_time << " us): " << info_);
    cmd_sync_t msg;
    msg.timestamp = current_time;
    send_message_to_device(CMD_SYNC, msg);
    last_sync_time_ = current_time;
}

long long SerialManager::get_timestamp_() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>
SerialManager::timestamp_to_chrono_(long long timestamp) {
    return std::chrono::time_point<std::chrono::system_clock,
                                   std::chrono::microseconds>(
        std::chrono::microseconds(timestamp));
}

bool SerialManager::is_wait_to_delete() { return wait_to_delete_.load(); }

const USBDeviceInfo& SerialManager::get_serial_info() const { return info_; }

void SerialManager::reset_device_() {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Reset device: " << info_);
    is_open_ = false;
    // send_message_to_device(CMD_RESET);
    // reset();
    wait_to_delete_ = true;
}

void SerialManager::publish_latency_(double microseconds) {
    std_msgs::msg::Float64 msg;
    msg.data = microseconds;
    latency_publisher_->publish(msg);
}

void SerialManager::create_communication_thread_() {
    read_thread_ = std::thread(&SerialManager::communication_thread_, this);

    // Set realtime policy
    sched_param param{10};
    int ret =
        pthread_setschedparam(read_thread_.native_handle(), SCHED_RR, &param);
    if (ret != 0) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Cannot set thread scheduling policy (SCHED_RR): "
                               << strerror(ret));
    }
}

void SerialManager::send_message_to_device(uint16_t cmd_id) {
    if (serial_alive_()) {
        uint8_t* frame_buffer = reinterpret_cast<uint8_t*>(
            alloca(protocol_calculate_frame_size(0)));
        const size_t frame_size =
            protocol_pack_data_to_buffer(cmd_id, nullptr, 0, frame_buffer);
        write(frame_buffer, frame_size);
    }
}

bool SerialManager::open_device_() {
    if (init()) {
        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            CL_BOLDGREEN << "Open humanoid serial device success: " << info_
                         << CL_RESET);
        is_open_ = true;
        return true;
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "Cannot open humanoid serial device: " << info_);
        return false;
    }
}

void SerialManager::initialize_device_() {
    // Sync time.
    sync_time_();

    // Read board app and initialize motor if needed
    send_message_to_device(CMD_READ_APP_ID);
    long long read_app_id_last_time = get_timestamp_();
    while (serial_alive_() && read_app_id_last_time > 0) {
        read_and_parse_([&]() {
            if (unpack_stream_obj_->cmd_id == CMD_READ_APP_ID_FEEDBACK &&
                unpack_stream_obj_->data_len ==
                    sizeof(cmd_read_app_id_feedback_t)) {
                app_id_ = reinterpret_cast<cmd_read_app_id_feedback_t*>(
                              unpack_stream_obj_->data)
                              ->app_id;

                // Initialize Leg Motor
                if (app_id_ == 2 || app_id_ == 3 || app_id_ == 4) {
                    send_message_to_device(CMD_INITIALIZE_MOTOR);
                }
                read_app_id_last_time = -1;
            } else {
                // Read app id timeout, retry
                if (get_timestamp_() - read_app_id_last_time > 1e6) {
                    send_message_to_device(CMD_READ_APP_ID);
                    read_app_id_last_time = get_timestamp_();
                }
            }
        });
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    // Check if faliure
    if (read_app_id_last_time > 0) {
        reset_device_();
    }
}

void SerialManager::read_and_parse_(std::function<void(void)> callback) {
    // Read from usb
    long long ret = read(read_buffer_.data(), read_buffer_.size());
    if (ret > 0) read_last_time_ = get_timestamp_();

    // Unpack data frame
    for (long long i = 0; i < ret; ++i) {
        if (protocol_unpack_byte(unpack_stream_obj_, read_buffer_[i])) {
            publish_latency_(get_timestamp_() - (*reinterpret_cast<long long*>(
                                                    unpack_stream_obj_->data)));
            callback();
        }
    }
}

bool SerialManager::serial_alive_() {
    return rclcpp::ok() && is_open_ &&
           (get_timestamp_() - read_last_time_ < read_timeout_);
}

void SerialManager::communication_thread_() {
    // Open port.
    if (!open_device_()) {
        wait_to_delete_ = true;
        return;
    }

    // Initialize device
    initialize_device_();

    // Unpack data from device
    while (serial_alive_()) {
        long long current_timestamp = get_timestamp_();

        // Sync device time
        if (current_timestamp - last_sync_time_ > sync_duration_) {
            sync_time_();
        }

        // Read and parse stream
        read_and_parse_([this]() {
            // Zero copy
            node_->dispatch_frame_(
                shared_from_this(), unpack_stream_obj_->cmd_id,
                unpack_stream_obj_->data, unpack_stream_obj_->data_len);
        });

        // 1 ms period
        std::this_thread::sleep_until(
            timestamp_to_chrono_(current_timestamp + 1000));
    }
}
}  // namespace humanoid