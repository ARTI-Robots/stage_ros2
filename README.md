# stage_ros2

[![Join the chat at https://gitter.im/stage_ros2/community](
 https://badges.gitter.im/stage_ros2/community.svg)](
 https://gitter.im/stage_ros2/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

ROS 2 node wrapping the [Stage](https://github.com/rtv/Stage/) simulator.


## License

The stage_ros2 package is GPL-licensed, as Stage is GPL-licensed and stage_ros2 links to it. See
[stage_ros2/LICENSE](stage_ros2/LICENSE) for details.

The stage_ros2_itfs package is BSD-licenced (Simplified BSD License). See
[stage_ros2_itfs/LICENSE](stage_ros2_itfs/LICENSE) for details.


## Build

```
cd ros2_ws/src
git clone https://github.com/rtv/Stage.git
git clone https://github.com/ymd-stella/stage_ros2.git
cd ..
colcon build --symlink-install
source install/setup.sh
```


## Use

```
ros2 run stage_ros2 stage_ros2 --ros-args -p world:=camera.world
```

ROS parameters:

- world: path to the world file.
- gui: whether to show the Stage GUI.
- &lt;model_name&gt;/is_depth_canonical: whether to publish depth image in float32 (true) or 
  uint16 (false) format.
