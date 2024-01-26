#!/bin/env bash

set -euxo pipefail

echo 'ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"' > /etc/udev/rules.d/70-humanoid-chassis.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
