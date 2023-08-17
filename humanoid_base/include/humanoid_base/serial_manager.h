#ifndef __SERIAL_MANAGER_H__
#define __SERIAL_MANAGER_H__

#include <sensor_msgs/msg/imu.h>

#include <atomic>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>

#include "humanoid_base/protocol.h"
#include "humanoid_base/usb_device.h"

namespace humanoid {

class HumanoidBaseNode;

class SerialManager : public std::enable_shared_from_this<SerialManager>,
                      public USBDevice {
public:
    SerialManager(const USBDeviceInfo& i, HumanoidBaseNode* const node,
                  long long sync_duration = 1000000,
                  long long read_timeout = 1000000);
    ~SerialManager();

    bool is_wait_to_delete();

    void send_message_to_device(uint16_t cmd_id);

    template <typename T>
    void send_message_to_device(uint16_t cmd_id, const T& msg) {
        if (serial_alive_()) {
            uint8_t* frame_buffer = reinterpret_cast<uint8_t*>(
                alloca(protocol_calculate_frame_size(sizeof(T))));
            const size_t frame_size = protocol_pack_data_to_buffer(
                cmd_id, reinterpret_cast<const uint8_t*>(&msg), sizeof(T),
                frame_buffer);
            write(frame_buffer, frame_size);
        }
    }

    const USBDeviceInfo& get_serial_info() const;

    uint8_t get_board_app_id();

private:
    std::atomic_bool is_open_;
    std::atomic_bool is_sync_;
    std::atomic_bool wait_to_delete_;
    std::thread read_thread_;
    HumanoidBaseNode* const node_;
    long long sync_duration_;
    long long read_timeout_;
    long long last_sync_time_;
    uint8_t app_id_;
    protocol_stream_t* unpack_stream_obj_;
    std::vector<uint8_t> read_buffer_;
    long long read_last_time_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;

    void create_communication_thread_();
    void sync_time_();
    void reset_device_();
    inline long long get_timestamp_();
    std::chrono::time_point<std::chrono::steady_clock,
                            std::chrono::microseconds>
    timestamp_to_chrono_(long long timestamp);
    void publish_latency_(double microseconds);
    bool open_device_();
    void initialize_device_();
    void read_and_parse_(std::function<void(void)> callback);
    bool serial_alive_();
    void communication_thread_();
};

}  // namespace humanoid

#endif  // __SERIAL_MANAGER_H__
