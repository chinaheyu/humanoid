# humanoid

Humanoid robot ros2 package

- humanoid_interface: Definition of message types;
- humanoid_base: Driver package for head and leg;
- humanoid_arm: Driver package for arm;
- humanoid_chat: Chat with LLM;
- humanoid_web: Restful api for frontend.

## Quick Start (docker)

The fastest way to start the whole robot is using docker image.

```bash
# launch ros package
docker run --name humanoid \
--restart unless-stopped \
--privileged --device=/dev:/dev \
-p 5000:5000 \
-e AZURE_SPEECH_KEY=$AZURE_SPEECH_KEY \
-e AZURE_SPEECH_REGION=$AZURE_SPEECH_REGION \
-e IFLYTEK_APP_ID=$IFLYTEK_APP_ID \
-e IFLYTEK_APP_KEY=$IFLYTEK_APP_KEY \
-e IFLYTEK_APP_SECRET=$IFLYTEK_APP_SECRET \
-d chinaheyu/humanoid

# launch web frontend
docker run --name humanoid_frontend --restart unless-stopped -p 80:80 -d chinaheyu/humanoid_frontend
```

## Quick Start (source)

Create a ros2 workspace.

```bash
mkdir -p ~/ros2_ws/src
```

Clone git Repository.

```bash
cd ~/ros2_ws/src
git clone https://github.com/chinaheyu/humanoid.git
```

Install dependency.

```bash
cd ~/ros2_ws/src/humanoid
./install_dependency.sh
```

Build workspace.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Setup UDEV rule and scheduler permission.

```bash
cd ~/ros2_ws/src/humanoid/humanoid_base/scripts
sudo ./setup_udev_rules.sh
./setup_scheduler_permission.sh
cd ~/ros2_ws/src/humanoid/humanoid_arm/scripts
sudo ./setup_udev_rules.sh
```

Note that the scheduler permission settings will take effect after a reboot. You can check if the `ulimit -r` command outputs `98`.

Finally, start all ros packages.

```bash
source "~/ros2_ws/install/setup.bash"
ros2 launch humanoid_bringup bringup.py
```

Optionally, you can use systemd to create a upstart service. This involves the following steps.

1. Create environment file.

~/.config/environment.d/humanoid.conf

```
AZURE_SPEECH_KEY=
AZURE_SPEECH_REGION=
IFLYTEK_APP_ID=
IFLYTEK_APP_KEY=
IFLYTEK_APP_SECRET=
ROS_DOMAIN_ID=1
```

2. Create service file.

~/.config/systemd/user/humanoid.service

```
[Unit]
After=network-online.target
Description="Humanoid launch"

[Service]
ExecStart=<path-to-launch.sh>
RemainAfterExit=no
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=default.target
```

3. Enable unit

```bash
systemctl --user enable humanoid.service
systemctl --user start humanoid.service
```

4. Check service output

```bash
journalctl -f --user-unit humanoid.service
```
