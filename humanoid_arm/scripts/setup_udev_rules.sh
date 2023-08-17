#!/bin/env bash

set -euxo pipefail

echo 'SUBSYSTEM=="tty", ATTRS{manufacturer}=="mjbots", ATTRS{product}=="fdcanusb", MODE="0666", SYMLINK+="fdcanusb"' > /etc/udev/rules.d/70-fdcanusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
