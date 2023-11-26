FROM ros:foxy

# Fix ros key expired
RUN rm /etc/apt/sources.list.d/ros2-snapshots.list && \
    apt update && apt install -y lsb-release wget curl software-properties-common gnupg && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    rm -rf /var/lib/apt/lists/*

# Install clangd 17 from llvm
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    apt-add-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-17 main" && \
    apt update && \
    apt install -y clangd-17 && \
    rm -rf /var/lib/apt/lists/*
