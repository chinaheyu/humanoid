# humanoid

Humanoid robot ros2 package

## Quick Start

TODO

## humanoid_interface

This package defines the types of messages, services and actions used by humanoid robots.

## humanoid_base

The underlying driver for the humanoid robot, used to communicate with the hardware.

The udev rule can be set up for the serial device to give readable and writable permissions.

```bash
sudo ./humanoid_base/setup_udev_rules.sh
```
