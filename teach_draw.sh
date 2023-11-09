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
echo "Teach mode enabled."

read -p "Calibrate arm and press Enter to continue..."
ros2 service call /arm/calibration std_srvs/srv/Empty "{}"

for i in {1..100}; do
    read -p "Press Enter to record current frame..."
    ros2 service call /arm/teach humanoid_interface/srv/TeachArm "frame_name: 'draw$i'"
    echo "draw$i is saved."
done
