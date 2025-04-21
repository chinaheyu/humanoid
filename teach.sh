#/usr/bin/bash

set -e

find_workspace_directory() {
    local current_dir=$(readlink -f "$0")

    while [[ "$current_dir" != "/" ]]; do
        if [[ -e "$current_dir/install/setup.bash" ]]; then
            echo "$current_dir"
            return 0
        fi
        current_dir="$(dirname "$current_dir")"
    done

    return -1
}

WORKSPACE_DIR=$(find_workspace_directory)

. "${WORKSPACE_DIR}/install/setup.bash"

ros2 service call /arm/teach_mode std_srvs/srv/SetBool "data: true"
ros2 service call /arm/teach humanoid_interface/srv/TeachArm "frame_name: '$1'"
ros2 service call /arm/get_frame_list humanoid_interface/srv/GetArmFrameList "{}"
