#/usr/bin/bash

set -e

handle_error() {
    echo "An error occurred in command: $BASH_COMMAND"
    exit 1
}

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

wait_for_path() {
    local path="$1"
    local interval="$2"
    
    while [ ! -e "$path" ]; do
        echo "Path $path does not exist. Waiting..."
        sleep "$interval"
    done
    
    echo "Path $path exists!"
}

wait_all_device_online() {
    wait_for_path "/dev/input/by-id/usb-DELI_DELI-14870_20080411-event-if03" 1
    wait_for_path "/dev/snd/by-id/usb-DELI_DELI-14870_20080411-00" 1
    wait_for_path "/proc/asound/DELI14870" 1

    wait_for_path "/dev/serial/by-id/usb-mjbots_fdcanusb_FFD4048A-if00" 1

    wait_for_path "/dev/serial/by-id/usb-scut_humanoid_205D32834D31-if00" 1
    wait_for_path "/dev/serial/by-id/usb-scut_humanoid_2064378F5948-if00" 1
    wait_for_path "/dev/serial/by-id/usb-scut_humanoid_205732834D31-if00" 1
    wait_for_path "/dev/serial/by-id/usb-scut_humanoid_206F32844D31-if00" 1
}

# catch error
trap 'handle_error' ERR

# wait for devices
wait_all_device_online
echo "All devices are connected!"

# set sound card volumn
amixer -q -c DELI14870 sset PCM 70%
amixer -q -c DELI14870 sset Mic 50%
echo "Set sound card volumn success!"

# find ros2 workspace
WORKSPACE_DIR=$(find_workspace_directory)

# launch ros2 packages
source "${WORKSPACE_DIR}/install/setup.bash"
ros2 launch humanoid_bringup bringup.py