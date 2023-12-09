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

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['show_direction1']
duration: [1]"

sleep 5

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['show_direction2', 'show_direction3']
duration: [1.5, 1.5]"

sleep 5

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['show_direction4', 'show_direction5', 'show_direction6']
duration: [1.5, 1, 1]"

sleep 5

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['show_direction7', 'show_direction8', 'show_direction9']
duration: [2, 1, 1]"

sleep 5

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['show_direction10', 'home']
duration: [1.5, 1]"
