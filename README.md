# glim_ros2

Please open issues in the main GLIM repository: https://github.com/koide3/glim

[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/build.yml)

## Hot to run with Autoware sample rosbag

![output](https://github.com/user-attachments/assets/65664fb9-718d-4503-9dcc-7abfd8618222)

Download links are quoted from [tutorial page](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/#rosbag-replay-simulation)

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1sU5wbxlXAfHIksuHjP3PyI2UVED8lZkP'
unzip -d ~/autoware_map/ ~/autoware_map/sample-rosbag.zip
```

Run command

```bash
ros2 run glim_ros glim_rosbag ~/autoware_map/sample-rosbag/ \
  --ros-args -p config_path:=$(ros2 pkg prefix glim_ros)/share/glim_ros/config
```
