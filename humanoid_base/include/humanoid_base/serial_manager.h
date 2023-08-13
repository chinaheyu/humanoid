#ifndef __SERIAL_MANAGER_H__
#define __SERIAL_MANAGER_H__

#include <pthread.h>
#include <sensor_msgs/msg/imu.h>

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "humanoid_base/protocol.h"
#include "humanoid_base/usb_device.h"

class HumanoidBaseNode;

class SerialManager : public std::enable_shared_from_this<SerialManager>,
                      public USBDevice {
public:
    SerialManager(const USBDeviceInfo& i, HumanoidBaseNode* const node,
                  long long sync_base_latency = 500,
                  long long sync_tolerance = 1000,
                  long long read_timeout = 100000);
    ~SerialManager();

    bool is_wait_to_delete();

    void send_message_to_device(uint16_t cmd_id);

    template <typename T>
    void send_message_to_device(uint16_t cmd_id, const T& msg) {
        if (rclcpp::ok() && is_open_) {
            uint8_t* frame_buffer = reinterpret_cast<uint8_t*>(
                alloca(protocol_calculate_frame_size(sizeof(T))));
            const size_t frame_size = protocol_pack_data_to_buffer(
                cmd_id, reinterpret_cast<const uint8_t*>(&msg), sizeof(T),
                frame_buffer);
            write(frame_buffer, frame_size);
        }
    }

    const USBDeviceInfo& get_serial_info() const;

private:
    std::atomic_bool is_open_;
    std::atomic_bool is_sync_;
    std::atomic_bool wait_to_delete_;
    pthread_t read_thread_;
    HumanoidBaseNode* const node_;
    long long sync_base_latency_;
    long long sync_tolerance_;
    long long read_timeout_;
    long long last_sync_time_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_publisher_;

    void create_read_thread_();
    void sync_time_();
    void reset_device_();
    inline long long get_timestamp_();
    void publish_latency_(double microseconds);

    friend void* read_from_serial_port_(void* obj);
};

#endif  // __SERIAL_MANAGER_H__
