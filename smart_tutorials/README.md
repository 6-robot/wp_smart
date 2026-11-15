# Smart Tutorials：串口底盘建图与导航

本包只有两个启动入口：`slam.launch.py` 和 `navigation.launch.py`，面向 `wp_smart` 的 ESP32-S3 串口固件。默认启动 `smart_bringup/bringup.launch.py`、机器人模型和 RViz，使用实时时钟，不启动仿真或独立雷达驱动。

## 数据链路与目录

底盘通过同一条串口发布 `/scan`、`/odom`、`/imu/data_raw` 和码盘数据，接收 `/cmd_vel`。建图与导航使用雷达和码盘里程计；底盘入口默认启动主机 Madgwick 节点发布 `/imu/data`，里程计由码盘积分。

TF 链为 `map → odom → base_footprint → base_link → laser_link → laser_scan_link`，`imu_link` 连接在 `base_link` 下。建图时由 slam_toolbox 发布 `map → odom`，导航时由 AMCL 发布，底盘驱动发布 `odom → base_footprint`。`/scan` 在朝后的 `laser_scan_link` 中定义后 0°、右 90°、前 180°、左 270°，TF 保持障碍物在车身坐标中的实际方向；沿用本包参数即可，无需修改建图或导航算法。

- `launch/`：建图、导航入口。
- `config/`：slam_toolbox 与 Nav2 参数。
- `rviz/`：地图、模型、Best Effort 雷达及导航显示。
- `maps/`：地图 YAML 与图像；实车导航使用与当前场地相符的地图。

## 安装与建图

以下命令从 colcon 工作空间根目录执行，先加载本机 ROS 2 Humble 环境：

```bash
rosdep install --from-paths src/wp_smart --ignore-src -r -y
colcon build --symlink-install --packages-up-to smart_tutorials
source install/setup.bash
ros2 launch smart_tutorials slam.launch.py
```

启动后使用遥控程序发送 `/cmd_vel`，缓慢移动底盘完成建图。另开终端加载工作空间后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f src/wp_smart/smart_tutorials/maps/map
```

## 导航

结束建图入口后再启动导航，默认加载本包 `maps/map.yaml` 及其引用的 `map.pgm`：

```bash
ros2 launch smart_tutorials navigation.launch.py
```

需要切换地图时可通过 `map` 参数指定其他地图 YAML。已有底盘驱动时加 `start_base:=false`，避免重复占用串口。

在 RViz 中使用 **2D Pose Estimate** 设置实车初始位置，再使用 **Nav2 Goal** 发送目标。不要同时运行建图和 AMCL，以免重复发布 `map → odom`。

底盘默认关闭指令超时刹车，并以 1 Hz 重发最后速度。需要在 ROS 指令中断后自动停车时，启动建图或导航入口加 `enable_cmd_vel_timeout:=true`。
