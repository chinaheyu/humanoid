#/usr/bin/bash

set -e

wait_network_online() {
    while true; do
        if ping -c 1 "southeastasia.api.cognitive.microsoft.com" > /dev/null 2>&1; then
            break
        else
            echo "Network is not online. Waiting..."
            sleep 1
        fi
    done
}

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

wait_for_usb() {
    local manufacturer="$1"
    local product="$2"
    local serial="$3"
    local interval="$4"
    local found=false

    while [ "$found" == false ]; do
        usb_devices=$(ls /dev/bus/usb/*/*)

        for usb_device in $usb_devices; do
            usb_info=$(udevadm info -q property -n $usb_device)

            if echo "$usb_info" | grep -q "ID_VENDOR=$manufacturer" && \
            echo "$usb_info" | grep -q "ID_MODEL=$product" && \
            echo "$usb_info" | grep -q "ID_SERIAL_SHORT=$serial"; then
                found=true
            fi
        done

        if [ "$found" == true ]; then
            echo "USB $manufacturer-$product-$serial exists!"
        else
            echo "USB $manufacturer-$product-$serial does not exist. Waiting..."
            sleep "$interval"
        fi
    done
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

wait_audio_online() {
    wait_for_path "/dev/input/by-id/usb-DELI_DELI-14870_20080411-event-if03" 1
    wait_for_path "/dev/snd/by-id/usb-DELI_DELI-14870_20080411-00" 1
    wait_for_path "/proc/asound/DELI14870" 1
}

wait_motors_online() {
    # arm
    wait_for_path "/dev/serial/by-id/usb-mjbots_fdcanusb_826543DB-if00" 1
    wait_for_path "/dev/serial/by-id/usb-mjbots_fdcanusb_1EB12734-if00" 1

    # head
    wait_for_usb scut humanoid 205D32834D31 1

    # waist
    # wait_for_usb scut humanoid 2064378F5948 1

    # left leg
    # wait_for_usb scut humanoid 205732834D31 1

    # right leg
    # wait_for_usb scut humanoid 206F32844D31 1
}

# catch error
trap 'handle_error' ERR

# sounds path
SOUNDS_DIR="$(dirname $(readlink -f "$0"))"/humanoid_bringup/sounds

# wait for audio device
wait_audio_online

# set sound card volumn
amixer -q -c DELI14870 sset PCM 100%
amixer -q -c DELI14870 sset Mic 80%
echo "Set sound card volumn success!"

# play notification sound
aplay -D sysdefault:CARD=DELI14870 "$SOUNDS_DIR"/sound1.wav

# wait for network
wait_network_online

# play notification sound
aplay -D sysdefault:CARD=DELI14870 "$SOUNDS_DIR"/sound2.wav

# wait for motor devices
wait_motors_online
echo "All devices are connected!"

# play notification sound
aplay -D sysdefault:CARD=DELI14870 "$SOUNDS_DIR"/sound3.wav

# find ros2 workspace
WORKSPACE_DIR=$(find_workspace_directory)

# launch ros2 packages
source "${WORKSPACE_DIR}/install/setup.bash"
# ros2 launch humanoid_bringup bringup.py
ros2 launch humanoid_bringup bringup_without_chat.py
