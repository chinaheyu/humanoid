#/bin/bash

set -e

SCRIPTPATH="$0"
TOP_DIR=$(dirname "$SCRIPTPATH")

# Sound card volumn
amixer -c DELI14870 sset PCM 70%
amixer -c DELI14870 sset Mic 50%

source "${TOP_DIR}/../../install/setup.bash"
echo `pwd`
ros2 launch humanoid_bringup bringup.py
