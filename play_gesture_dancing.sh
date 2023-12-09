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

ros2 service call /arm/play_sequence humanoid_interface/srv/PlayArmSequence "frame_name: ['gesture_dancing1', 'gesture_dancing2', 'gesture_dancing3', 'gesture_dancing4', 'gesture_dancing5', 'gesture_dancing6', 'gesture_dancing7', 'gesture_dancing8', 'gesture_dancing9', 'gesture_dancing10', 'gesture_dancing11', 'gesture_dancing12', 'gesture_dancing13', 'gesture_dancing14', 'gesture_dancing15', 'gesture_dancing16', 'gesture_dancing17', 'gesture_dancing18', 'gesture_dancing19', 'gesture_dancing20', 'gesture_dancing21', 'gesture_dancing22', 'gesture_dancing23', 'home']
duration: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]"
