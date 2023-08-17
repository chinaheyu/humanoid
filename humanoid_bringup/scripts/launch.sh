#/bin/bash

set -e

SCRIPTPATH=$(readlink -f "$0")
TOP_DIR=$(dirname "$SCRIPTPATH")

# Sound card volumn
amixer -c DELI14870 sset PCM 70%
amixer -c DELI14870 sset Mic 50%

source "${TOP_DIR}/../../../../install/setup.bash"
ros2 launch humanoid_bringup bringup.py
