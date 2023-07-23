#!/bin/env bash

set -euxo pipefail

echo 'SUBSYSTEM=="tty", ATTRS{manufacturer}=="scut", ATTRS{product}=="humanoid", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"' > /etc/udev/rules.d/humanoid.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
