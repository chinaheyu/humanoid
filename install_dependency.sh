#/bin/bash

set -e

SCRIPTPATH=$(readlink -f "$0")
TOP_DIR=$(dirname "$SCRIPTPATH")

rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths $TOP_DIR -r -y --ignore-src

install_requirements() {
    for dir in "$1"/*; do
        if [ -d "$dir" ]; then
            if [ -e "$dir/requirements.txt" ]; then
                echo "Installing requirements in $dir"
                pip3 install -r "$dir/requirements.txt"
            fi
            install_requirements "$dir"
        fi
    done
}
install_requirements $TOP_DIR
