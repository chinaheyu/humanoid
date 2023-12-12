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

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['hug1', 'hug2', 'hug3']
duration: [1, 1, 1]"

sleep 4

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['hug4']
duration: [1]"

sleep 2

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['hug5', 'hug6', 'home']
duration: [1, 1, 1]"