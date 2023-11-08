source install/setup.bash

ros2 service call /arm/teach_mode std_srvs/srv/SetBool "data: true"
ros2 service call /arm/teach humanoid_interface/srv/TeachArm "frame_name: '$1'"
ros2 service call /arm/get_frame_list humanoid_interface/srv/GetArmFrameList "{}"
