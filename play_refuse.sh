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

ros2 service call /arm/teach_mode std_srvs/srv/SetBool "data: false"
echo "Teach mode disable."

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['refuse1', 'refuse2', 'refuse3', 'refuse4', 'refuse5']
duration: [0.5, 0.5, 0.5, 0.5, 0.5]"

sleep 4

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['refuse6', 'refuse7', 'refuse8', 'refuse9', 'home']
duration: [0.5, 0.5, 0.5, 0.5, 0.5]"