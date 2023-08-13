#include "humanoid_base/usb_device.h"

LibUSBWarpper::LibUSBWarpper() : ctx_(nullptr) { libusb_init(&ctx_); }

LibUSBWarpper::~LibUSBWarpper() { libusb_exit(ctx_); }

std::vector<USBDeviceInfo> LibUSBWarpper::list_device() {
    std::vector<USBDeviceInfo> info_list;
    libusb_device **list = nullptr;
    int rc = 0;

    ssize_t count = libusb_get_device_list(ctx_, &list);

    for (ssize_t idx = 0; idx < count; ++idx) {
        libusb_device *device = list[idx];
        libusb_device_descriptor desc{};

        rc = libusb_get_device_descriptor(device, &desc);
        if (rc != LIBUSB_SUCCESS) continue;

        USBDeviceInfo info;
        info.device = device;
        info.vendor_id = desc.idVendor;
        info.product_id = desc.idProduct;

        try {
            libusb_device_handle *dev_handle;
            rc = libusb_open(device, &dev_handle);
            if (rc != LIBUSB_SUCCESS) continue;

            unsigned char data[256];
            int size;

            // manufacturer string
            size = libusb_get_string_descriptor_ascii(
                dev_handle, desc.iManufacturer, data, sizeof(data));
            if (size > 0)
                info.manufacturer =
                    std::string(reinterpret_cast<const char *>(data), size);

            // product string
            size = libusb_get_string_descriptor_ascii(dev_handle, desc.iProduct,
                                                      data, sizeof(data));
            if (size > 0)
                info.product =
                    std::string(reinterpret_cast<const char *>(data), size);

            // serial number string
            size = libusb_get_string_descriptor_ascii(
                dev_handle, desc.iSerialNumber, data, sizeof(data));
            if (size > 0)
                info.serial_number =
                    std::string(reinterpret_cast<const char *>(data), size);

            info_list.push_back(info);

            libusb_close(dev_handle);
        } catch (libusb_error &e) {
            continue;
        }
    }

    libusb_free_device_list(list, 1);
    return info_list;
}

USBDevice::USBDevice(const USBDeviceInfo &info, unsigned char in_ep,
                     unsigned char out_ep)
    : info_(info), in_ep_(in_ep), out_ep_(out_ep) {}

USBDevice::~USBDevice() {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
}

bool USBDevice::init() {
    if (libusb_open(info_.device, &dev_handle) != LIBUSB_SUCCESS) return false;
    if (libusb_set_auto_detach_kernel_driver(dev_handle, 1) != LIBUSB_SUCCESS)
        return false;
    if (libusb_claim_interface(dev_handle, 0) != LIBUSB_SUCCESS) return false;
    return true;
}

long long USBDevice::read(uint8_t *buf, size_t len) {
    int nread, ret;
    ret = libusb_bulk_transfer(dev_handle, in_ep_, buf, len, &nread,
                               0);
    if (ret != LIBUSB_SUCCESS) {
        return -1;
    }
    return nread;
}

long long USBDevice::write(uint8_t *buf, size_t len) {
    int nwrite, ret;
    ret = libusb_bulk_transfer(dev_handle, out_ep_, buf, len,
                               &nwrite, 0);
    if (ret != LIBUSB_SUCCESS) {
        return -1;
    }
    return nwrite;
}
