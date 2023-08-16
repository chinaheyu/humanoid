#include "humanoid_base/serial_manager.h"

#include <alloca.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>

#include "humanoid_base/humanoid_base_node.h"
#include "humanoid_base/protocol_def.h"
#include "humanoid_base/rich.h"

void* read_from_serial_port_(void* obj);

SerialManager::SerialManager(const USBDeviceInfo& i,
                             HumanoidBaseNode* const node,
                             long long sync_base_latency,
                             long long sync_tolerance, long long read_timeout)
    : USBDevice(i),
      is_open_(false),
      is_sync_(false),
      wait_to_delete_(false),
      node_(node),
      sync_base_latency_(sync_base_latency),
      sync_tolerance_(sync_tolerance),
      read_timeout_(read_timeout),
      last_sync_time_(0),
      app_id_(0) {
    latency_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "latency/s" + info_.serial_number, 10);
    create_read_thread_();
}

SerialManager::~SerialManager() {
    is_open_ = false;
    pthread_join(read_thread_, nullptr);
}

void SerialManager::sync_time_() {
    long long current_time = get_timestamp_();
    if (current_time - last_sync_time_ > 1000000) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            "Sync device time (timestamp = "
                                << current_time << " us): " << info_);
        cmd_sync_t msg;
        msg.timestamp = current_time;
        send_message_to_device(CMD_SYNC, msg);
        last_sync_time_ = current_time;
    }
}

long long SerialManager::get_timestamp_() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool SerialManager::is_wait_to_delete() { return wait_to_delete_.load(); }

const USBDeviceInfo& SerialManager::get_serial_info() const { return info_; }

void SerialManager::reset_device_() {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Reset device: " << info_);
    is_open_ = false;
    send_message_to_device(CMD_RESET);
    reset();
    wait_to_delete_ = true;
}

void SerialManager::publish_latency_(double microseconds) {
    std_msgs::msg::Float64 msg;
    msg.data = microseconds;
    latency_publisher_->publish(msg);
}

void SerialManager::create_read_thread_() {
    pthread_attr_t attr;
    sched_param param{10};

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedparam(&attr, &param);
    int ret =
        pthread_create(&read_thread_, &attr, read_from_serial_port_, this);
    if (ret != 0) {
        RCLCPP_WARN_STREAM(
            node_->get_logger(),
            "Cannot create thread (SCHED_RR): " << strerror(ret));
        pthread_create(&read_thread_, nullptr, read_from_serial_port_, this);
    }
}

void SerialManager::send_message_to_device(uint16_t cmd_id) {
    if (rclcpp::ok() && is_open_) {
        uint8_t* frame_buffer = reinterpret_cast<uint8_t*>(
            alloca(protocol_calculate_frame_size(0)));
        const size_t frame_size =
            protocol_pack_data_to_buffer(cmd_id, nullptr, 0, frame_buffer);
        write(frame_buffer, frame_size);
    }
}

void* read_from_serial_port_(void* obj) {
    SerialManager* serial = reinterpret_cast<SerialManager*>(obj);
    // Open port.
    if (serial->init()) {
        RCLCPP_INFO_STREAM(serial->node_->get_logger(),
                           CL_BOLDGREEN
                               << "Open humanoid serial device success: "
                               << serial->info_ << CL_RESET);
        serial->is_open_ = true;
    } else {
        RCLCPP_ERROR_STREAM(
            serial->node_->get_logger(),
            "Cannot open humanoid serial device: " << serial->info_);
        serial->wait_to_delete_ = true;
        return nullptr;
    }

    // Sync time.
    serial->sync_time_();

    // Read board app and initialize motor if needed
    serial->send_message_to_device(CMD_READ_APP_ID);
    long long read_app_id_last_time = serial->get_timestamp_();

    // Timeout detect
    long long device_read_last_time = serial->get_timestamp_();

    // Unpack message.
    protocol_stream_t* unpack_stream_obj =
        protocol_create_unpack_stream(1000, true);
    std::vector<uint8_t> buffer(4096, 0);
    long long ret;
    while (rclcpp::ok() && serial->is_open_) {
        ret = serial->read(buffer.data(), buffer.size());
        if (ret > 0) {
            device_read_last_time = serial->get_timestamp_();
        } else {
            if (serial->get_timestamp_() - device_read_last_time > 1e6) {
                serial->reset_device_();
            }
        }
        for (long long i = 0; i < ret; ++i) {
            if (protocol_unpack_byte(unpack_stream_obj, buffer[i])) {
                // Read board app
                if (read_app_id_last_time > 0) {
                    if (unpack_stream_obj->cmd_id == CMD_READ_APP_ID_FEEDBACK &&
                        unpack_stream_obj->data_len ==
                            sizeof(cmd_read_app_id_feedback_t)) {
                        serial->app_id_ =
                            reinterpret_cast<cmd_read_app_id_feedback_t*>(
                                unpack_stream_obj->data)
                                ->app_id;

                        // Initialize Leg Motor
                        if (serial->app_id_ == 2 || serial->app_id_ == 3 ||
                            serial->app_id_ == 4) {
                            serial->send_message_to_device(
                                CMD_INITIALIZE_MOTOR);
                        }

                        read_app_id_last_time = -1;
                    } else {
                        if (serial->get_timestamp_() - read_app_id_last_time >
                            1e6) {
                            serial->send_message_to_device(CMD_READ_APP_ID);
                            read_app_id_last_time = serial->get_timestamp_();
                        }
                    }
                }

                // Sync time.
                long long device_time =
                    *reinterpret_cast<long long*>(unpack_stream_obj->data);
                long long err = serial->get_timestamp_() - device_time;
                serial->publish_latency_(err);
                if (err >
                        serial->sync_base_latency_ + serial->sync_tolerance_ ||
                    err <
                        serial->sync_base_latency_ - serial->sync_tolerance_) {
                    RCLCPP_DEBUG_STREAM(serial->node_->get_logger(),
                                        "Sync device timestamp faliure (err = "
                                            << err
                                            << " us): " << serial->info_);
                    serial->is_sync_ = false;
                    serial->sync_time_();
                } else if (!serial->is_sync_) {
                    serial->is_sync_ = true;
                }

                // Received frame.
                if (serial->is_sync_) {
                    // One time copy.
                    serial->node_->dispatch_frame_(
                        serial->shared_from_this(), unpack_stream_obj->cmd_id,
                        std::make_shared<std::vector<uint8_t>>(
                            unpack_stream_obj->data,
                            unpack_stream_obj->data +
                                unpack_stream_obj->data_len));
                }
            }
        }

        usleep(1000);
    }
    protocol_free_unpack_stream(unpack_stream_obj);
    return nullptr;
}
