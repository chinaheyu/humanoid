#ifndef __USB_DEVICE_H__
#define __USB_DEVICE_H__

#include <libusb-1.0/libusb.h>

#include <iomanip>
#include <string>
#include <vector>

namespace humanoid {

struct USBDeviceInfo {
    libusb_device* device;
    unsigned short product_id;
    unsigned short vendor_id;
    std::string product;
    std::string manufacturer;
    std::string serial_number;

    friend std::ostream& operator<<(std::ostream& os, const USBDeviceInfo& d) {
        os << std::setfill('0') << std::setw(4) << std::right << std::hex
           << d.product_id << ":" << std::setfill('0') << std::setw(4)
           << std::right << std::hex << d.vendor_id << ", " << d.manufacturer
           << ", " << d.product << ", " << d.serial_number;
        return os;
    }
};

class LibUSBWarpper {
public:
    static LibUSBWarpper& get_instance() {
        static LibUSBWarpper instance;
        return instance;
    }
    LibUSBWarpper(LibUSBWarpper const&) = delete;
    void operator=(LibUSBWarpper const&) = delete;

    std::vector<USBDeviceInfo> list_device();

private:
    LibUSBWarpper();
    ~LibUSBWarpper();

    libusb_context* ctx_;
};

class USBDevice {
public:
    USBDevice(const USBDeviceInfo& info, unsigned char in_ep = 0x81U,
              unsigned char out_ep = 0x01U, unsigned char cmd_ep = 0x82U);
    ~USBDevice();

    bool init();

    long long read(uint8_t* buf, size_t len);

    long long write(uint8_t* buf, size_t len);

    bool reset();

protected:
    const USBDeviceInfo info_;

private:
    libusb_device_handle* dev_handle_;
    unsigned char in_ep_;
    unsigned char out_ep_;
    unsigned char cmd_ep_;
};

}  // namespace humanoid

#endif  // __USB_DEVICE_H__
