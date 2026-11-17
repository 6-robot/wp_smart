# WP Smart 串口底盘

ESP32-S3 通过一条 UART 与 `smart_bringup` 联动，承载双差动电机控制、码盘、MPU-60x0 原始 IMU 和 YDLIDAR X2N 雷达数据。Wi-Fi 方案独立保存在相邻的 `smart_wifi` 目录。

## 目录

- `firmware/smart_esp32_serial/`：本方案的 Arduino 固件，烧录此目录。
- `smart_bringup/`：C++ 串口驱动、协议、参数、启动文件及测试。
- `smart_description/`：机器人模型、雷达和 IMU 坐标系。
- `smart_tutorials/`：串口底盘的 SLAM 建图与 Nav2 导航，各保留一个启动入口。
- `smart_bringup/doc/`：协议与参数索引、独立串口检查工具。
- `docs/`：[串口协议](docs/serial_protocol.md)与[底盘参数](docs/小车物理参数.md)。

## USB 与接线

底盘使用两个 USB 接口，职责如下；设备编号可能随插拔顺序变化：

| 用途 | 设备类型 | USB 标识 | 序列号 |
| --- | --- | --- | --- |
| 烧录、115200 调试日志 | `ttyUSB` | CP2102 `10c4:ea60` | `0001` |
| 底盘通讯、460800 8N1 | `ttyACM` | WCH `1a86:55d4` | `5C85476222` |

`install_udev_rule.sh` 只安装当前 WCH 底盘通讯口规则；雷达与 IMU 共用该串口，无需单独的雷达设备映射。

### 安装底盘 udev 规则

`smart_bringup/udev/smart_base.rules` 按 WCH 的厂商 ID `1a86`、产品 ID `55d4` 和序列号 `5C85476222` 匹配通讯口，生成 `smart_base` 设备别名。

从工作空间根目录执行：

```bash
# 先检查匹配结果，不需要 sudo，也不会修改系统或读写串口
src/wp_smart/smart_bringup/scripts/install_udev_rule.sh --dry-run

# 安装时输入 sudo 密码
sudo src/wp_smart/smart_bringup/scripts/install_udev_rule.sh
```

脚本读取规则中的设备身份，查找匹配通讯口，备份系统规则并处理同名别名冲突，然后重载 udev、验证别名；失败时恢复备份。

脚本支持 `--device` 指定通讯设备路径。更换适配器后，更新 `smart_base.rules` 中的设备身份，再用 `--dry-run` 核对。

安装后，启动时可把生成的设备别名路径传给 `port`。若未安装别名，省略 `port` 参数即可使用默认 by-id 路径。修改工程文件或仅加载工作空间不会自动创建系统设备别名。

| 功能 | ESP32-S3 引脚 |
| --- | --- |
| 左/右 DRV8870 IN1、IN2 | 11、12 / 13、14 |
| 左/右码盘 A、B | 4、5 / 6、7 |
| 通讯 UART1 TX、RX | 17、18，连接转换器 RX、TX，双方共地 |
| X2N UART2 RX、TX / 电机 PWM | 47、48 / 21 |
| IMU、OLED 共用 I²C SDA、SCL | 9、10；IMU `0x68`、OLED `0x3C` |
| 电池 ADC | 1 |
| WS2812B 灯带数据（6 颗，GRB、800 Kbps） | 45，经板载电平转换连接 DIN |

## 编译与烧录

以下命令从包含 `src/wp_smart` 的 colcon 工作空间根目录执行。使用 ROS 2 Humble，先加载本机 ROS 环境。固件使用 **Espressif ESP32 core 2.0.17** 和 **Adafruit NeoPixel 1.15.5**，不需要升级 Core。

```bash
rosdep install --from-paths src/wp_smart --ignore-src -r -y
colcon build --symlink-install --packages-up-to smart_tutorials
source install/setup.bash

arduino-cli lib install 'Adafruit NeoPixel@1.15.5'
read -r -p '烧录串口设备路径: ' FLASH_PORT
arduino-cli compile --fqbn esp32:esp32:esp32s3:UploadSpeed=115200,PSRAM=disabled \
  src/wp_smart/firmware/smart_esp32_serial
arduino-cli upload --port "$FLASH_PORT" \
  --fqbn esp32:esp32:esp32s3:UploadSpeed=115200,PSRAM=disabled \
  src/wp_smart/firmware/smart_esp32_serial
```

## 启动与数据检查

```bash
ros2 launch smart_bringup bringup.launch.py
# 更换通讯适配器时：
read -r -p '通讯串口设备路径: ' COMM_PORT
ros2 launch smart_bringup bringup.launch.py port:="$COMM_PORT"

ros2 topic hz /scan
ros2 topic hz /imu/data_raw
ros2 topic echo /encoder_counts
ros2 topic echo /diagnostics
```

不要同时启动两次驱动，或同时用串口工具占用通讯口。驱动默认发布机器人模型；已有模型发布器时加 `publish_model:=false`。`imu_xyz`、`imu_rpy` 是 IMU 相对于 `base_link` 的安装位置和方向，当前零值是待测占位，使用融合前必须按实物设置。雷达沿用现有模型安装位置。

### USB 摄像头与底盘一起启动

`camera_bringup.launch.py` 同时启动 `bringup.launch.py` 和 `usb_cam`，默认选择 `video0`，使用 MJPEG 转 RGB、640×480，请求帧率 30 FPS。实际帧率取决于摄像头、曝光和主机负载。先安装驱动依赖：

```bash
sudo apt install ros-humble-usb-cam
```

构建并加载工作空间后执行：

```bash
ros2 launch smart_bringup camera_bringup.launch.py
# 摄像头支持 MJPEG 720p 时：
ros2 launch smart_bringup camera_bringup.launch.py camera_width:=1280 camera_height:=720
```

此入口已启动底盘，不要再单独运行 `bringup.launch.py`。`port`、`baud_rate`、`enable_cmd_vel_timeout`、IMU 及模型参数沿用底盘入口，例如追加 `enable_cmd_vel_timeout:=true`。

底盘未连接或已由其他入口启动时，使用 `start_base:=false` 只启动摄像头：

```bash
ros2 launch smart_bringup camera_bringup.launch.py start_base:=false
```

通过 `camera_device` 选择目标相机；多摄像头环境建议指定对应 by-id 图像接口，入口在启动时解析为真实视频节点。默认 `video0` 不按厂商筛选设备。若底盘提示通讯口不存在，应连接 WCH 通讯 USB，或通过 `port` 指定实际通讯设备。

默认图像话题为 `/camera/image_raw`，内参话题为 `/camera/camera_info`；在 RViz 中添加 **Image** 显示并选择图像话题即可预览。也可运行 `ros2 run rqt_image_view rqt_image_view`。

可覆盖 `camera_device`、`camera_namespace`、`camera_frame_id`、`camera_width`、`camera_height`、`camera_framerate`、`camera_pixel_format` 和 `camera_info_url`。更换设备时选择其 `video-index0` 图像接口；`camera_info_url` 用于加载相机标定 YAML。尚未标定时，驱动可能提示标定文件不存在，仍可预览图像，此时 `camera_info` 不含有效标定内参。图像默认使用 `camera_optical_frame`，本入口不添加未经测量的相机安装 TF；需要视觉定位或投影时应补充实测安装变换和相机内参。

### 雷达 RViz 与 IMU 终端显示

已有构建产物且仅修改 launch、RViz 配置或 Python 脚本时，可单独安装资源：

```bash
cmake -S src/wp_smart/smart_bringup -B build/smart_bringup
cmake --install build/smart_bringup
source install/setup.bash
```

`source` 只加载环境；添加启动文件后还需构建或安装资源。

完成上面的 ROS 包构建并加载 `install/setup.bash` 后，分别使用：

```bash
# 启动串口驱动、机器人模型，并用 RViz 点样式显示雷达 /scan
ros2 launch smart_bringup scan_rviz.launch.py

# 启动串口驱动、机器人模型，在当前终端显示 IMU 角度（单位：度）
ros2 launch smart_bringup imu_console.launch.py
```

每个入口默认启动底盘驱动。已经运行 `bringup.launch.py` 或雷达预览时，第二个终端加 `start_base:=false`，避免重复打开通讯串口：

```bash
ros2 launch smart_bringup imu_console.launch.py start_base:=false
```

两个入口均可沿用 `bringup.launch.py` 的 `port`、`baud_rate`、`params_file`、`imu_xyz`、`imu_rpy` 参数。RViz 默认固定坐标系为 `base_link`，使用 Best Effort QoS 接收 `/scan`，可通过 `rviz_config` 指定自己的显示配置。

IMU 终端默认订阅 `/imu/data`，每秒显示 5 次横滚、俯仰和航向角。启动底盘后保持静止约 2 秒，让融合节点估计陀螺仪零偏；校准期间暂不发布融合数据。这里的航向始终是相对值，会漂移，终端中的 `yaw` 不代表绝对朝向。

```bash
ros2 launch smart_bringup imu_console.launch.py start_base:=false display_rate:=10.0
# 调试原始数据时，可切换到终端自身的简易姿态估计：
ros2 launch smart_bringup imu_console.launch.py start_base:=false \
  imu_topic:=/imu/data_raw calibration_seconds:=3.0
```

终端超过 0.5 秒无新数据时显示等待提示，不继续输出旧角度。`calibration_seconds`、`correction_gain` 只控制终端读取原始话题时的简易估计；正式融合节点的配置见下节。

### IMU 姿态融合

`bringup.launch.py` 默认启动 `smart_imu_filter_node`，调用官方 [imu_filter_madgwick](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick) 的 Madgwick 六轴算法库，数据链路为：

```text
ESP32 → /imu/data_raw → 静止零偏估计 + Madgwick → /imu/data
```

- `/imu/data_raw` 保留未标定的加速度、角速度，姿态不可用。
- `/imu/data` 包含归一化姿态四元数，角速度减去启动时估计的零偏；加速度保留重力及传感器偏置。时间戳、传感器坐标系和加速度/角速度协方差沿用输入。
- 按 ENU 的向上重力方向初始化横滚、俯仰，初始相对航向为零。没有磁力计，航向没有绝对参考；直线加速可能影响倾角。输出仍属于 `imu_link`，安装方向通过 `imu_xyz`、`imu_rpy` 对应的 TF 描述。
- 默认连续静止至少 2 秒且至少收到 50 个样本后输出。检测到明显运动、采样间断或无效样本会重新累计校准。恒定低速转动无法仅靠六轴数据可靠区分零偏，启动时必须保持静止。
- 校准完成后，采样间隔超过 0.2 秒或时间回退会保留零偏、重新初始化倾角并将相对航向归零；`frame_id` 改变会重新估计零偏。输入停更时不重复发布旧姿态。
- 输入采用 Sensor Data QoS，输出采用 Reliable、深度 10，可供普通 `sensor_msgs/Imu` 订阅者使用。节点不发布 TF 或速度指令，现有 `/odom` 仍由码盘积分。

配置文件为 `smart_bringup/config/imu_filter.yaml`：`calibration_seconds`、`min_calibration_samples` 控制启动校准，`gain` 默认 0.1，`max_sample_gap` 默认 0.2 秒。`calibration_seconds: 0.0` 可跳过零偏估计。`orientation_stddev: 0.0` 表示姿态协方差未知；需要协方差的后续融合器应按实测噪声设置，六轴航向也不能作为无漂移的绝对方位。

融合参数在启动时加载，修改 YAML 后重启节点。通过 `imu_filter_params_file` 指定自定义融合 YAML；`use_imu_filter:=false` 可关闭底盘入口自带的融合节点。已有原始 IMU 发布器而尚无融合节点时，可单独启动，不打开串口：

```bash
ros2 launch smart_bringup imu_filter.launch.py
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

不要同时启动两个融合节点发布同一话题。使用本方案的 SLAM、导航和雷达预览入口时，也会随底盘入口默认启动融合。

依赖 `imu_filter_madgwick` 可随前述 `rosdep install` 安装。没有系统安装权限时，可从工作空间根目录在用户目录内构建官方 Humble 分支：

```bash
git clone --branch humble --depth 1 https://github.com/CCNYRoboticsLab/imu_tools.git .deps/imu_tools
colcon build --symlink-install --base-paths src .deps/imu_tools/imu_filter_madgwick \
  --packages-up-to smart_tutorials
source install/setup.bash
```

只显式发现依赖中的 `imu_filter_madgwick` 包。融合节点及算法封装使用 GPL-3.0-or-later，许可全文位于 `smart_bringup/LICENSE.imu_fusion`；各源文件按其许可声明使用。

### ROS 接口与底盘参数

| 接口 | 内容 |
| --- | --- |
| `/cmd_vel` | `Twist`：前进 m/s、左转 rad/s；限幅 0.5 m/s、2.5 rad/s |
| `/encoder_counts` | `Int32MultiArray`：左、右原始累计计数；右轮前进通常为负 |
| `/odom`、TF | 码盘积分；右轮符号由参数修正；未融合 IMU |
| `/imu/data_raw` | m/s²、rad/s；保留重力和传感器偏置，姿态不可用 |
| `/imu/data` | 主机 Madgwick 姿态四元数、零偏校正角速度；航向为相对值 |
| `/scan` | Reliable、深度 10；`laser_scan_link` 中后 0°、右 90°、前 180°、左 270°，450 槽 |
| `/battery_state`、`/diagnostics` | 电压估计、通讯统计及数据新鲜度 |


`/imu/data` 和串口驱动的 `/scan` 均采用 Reliable、深度 10，普通 C++ 订阅即可接收：

```cpp
auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
```

`SensorDataQoS()`（Best Effort）订阅兼容这两个 Reliable 发布者。`/imu/data_raw` 使用 Sensor Data QoS，应采用 Best Effort 订阅。

固件和主机默认轮径 **0.0657 m**、轮距 **0.1805 m**、减速比 **64:1**、每轮 **262144** 计数。修改轮径或轮距时必须同步固件宏和 `smart_base.yaml`。电池电压为 ADC 分压估计值，应与万用表核对。

### 速度保持与超时刹车开关

驱动、`smart_base.yaml` 和 `bringup.launch.py` 的 `enable_cmd_vel_timeout` 默认均为 `false`，即关闭 ROS 指令超时刹车：

- 收到有效 `/cmd_vel` 后保留最后的线速度、角速度；新指令及时转发，无新指令时每秒重发一次，不受 `cmd_vel_timeout` 限制。
- ESP32 的串口指令超时保护为 3 秒，1 Hz 重发会持续更新其期限。即使 ROS 速度发布者停止，只要驱动继续运行且串口连通，底盘仍保持最后速度，需显式发送零速才能停车。
- 启动后没有速度指令时，驱动只重发零速；收到零速会及时转发并持续保持零速。驱动正常退出仍发送零速帧；串口重连会清除旧速度，等待新的 `/cmd_vel`，不自动恢复之前的运动。
- `enable_cmd_vel_timeout:=true` 时，主机超过 `cmd_vel_timeout`（默认 3.0 秒）未收到新指令后发送零速，按 `command_rate`（默认 50 Hz）发送控制帧。固件在独立 100 Hz 任务中执行其 3 秒期限检查。

```bash
# 默认：关闭超时刹车，最后速度每秒重发
ros2 launch smart_bringup bringup.launch.py
# 开启主机指令超时刹车
ros2 launch smart_bringup bringup.launch.py enable_cmd_vel_timeout:=true
```

此开关为只读启动参数，变更需重启驱动；`/diagnostics` 的 `Command timeout enabled` 可查看当前模式。雷达预览、IMU 显示以及建图、导航入口启动底盘时也沿用此参数。显式零速、正常退出和串口故障期间 ESP32 自身的超时保护不受开关影响。

ESP32 上电时车轮保持零速，GPIO21 的雷达 PWM 为 0。收到有效串口速度帧（包括零速）后，雷达 PWM 输出 110/255（10 kHz，LEDC 通道 4）；满 3 秒未收到有效速度帧时关闭 PWM，收到新帧后恢复。显式 `0x02` 停止帧同时关闭雷达 PWM，实际停稳存在机械惯性。电机 PWM 保持 20 kHz、LEDC 通道 0–3。

底盘驱动启动后会持续发送速度帧，因此车轮静止时也能扫描；停止发布 `/cmd_vel` 不会关闭雷达。驱动退出或串口断开后，最后一帧速度满 3 秒才停转。非零速联调前确认电池供电并将驱动轮架空。

### 雷达角度与屏幕状态

串口驱动将修正后的 X2N 原始角度变换为 `(-原始角度) mod 360°`。`/scan` 的 `angle_min=0`，以车尾为 0°，右侧 90°、车头 180°、左侧 270°。消息使用 `laser_scan_link`，相对 `laser_link` 绕 Z 旋转 180°；TF 将点还原到车身方向。自定义模型必须包含该变换。

板载 SSD1306 OLED 状态页（128×64，I²C 地址 `0x3C`）显示如下内容，数值为示例：

```text
WP-SMART 2026111707
BAT:11.80V
FRONT:1.23m
L ENC:123456
R ENC:-123456
IMU:OK    LIDAR:OK
```

- 首行显示 `WP-SMART` 和 `FIRMWARE_VERSION` 固件版本号，与串口状态帧和启动日志使用同一版本宏；以后更新版本宏即可同步屏幕显示，不是实时时钟。
- 第二行显示电池电压；无有效电压时显示 `BAT:--V`。电压分压校正需实测核对。
- `FRONT` 是以雷达为起点、车头方向 ±5° 内的最近有效距离，过滤 0.24–10 m 以外的回波。仅使用完整有效扫描；该扇区没有回波或扫描超过 1 秒未更新时显示 `--`，不保留旧距离。雷达转动期间由固件直接更新，无需 ROS 订阅。
- 左右码盘显示与 `/encoder_counts` 一致的原始有符号累计计数，右轮前进通常为负。
- 最后一行的 IMU 状态与雷达状态之间固定间隔 4 个 ASCII 空格。
- 状态页每 500 ms 取样一次，每次主循环最多发送 16 字节像素；I²C 与 IMU 在主循环中顺序使用。屏幕缺失或刷新出错时停止屏幕传输，电机 PID 和 3 秒超时停车仍在独立任务中运行。

IMU 使用 [InvenSense MPU-60x0 寄存器说明](https://www.electrokit.com/upload/product/41013/41013621/RM-MPU-60xxA_rev_4.pdf) 的量程换算。原始输出没有零偏、尺度或安装方向标定，不能直接作为已融合姿态使用。IMU 时间戳为主机收包时刻；雷达为整圈接收时刻，未做时钟同步或运动去畸变。

## WS2812B 灯带

固件通过 GPIO45 控制 6 颗 WS2812B，亮度上限为 32/255。上电等待指令和指令超时后，按红、黄、绿、青、蓝、品红循环呼吸，每 2 秒平滑过渡到下一种颜色，保留非零亮度，不出现全灭间隔；正常零速或显式停止保持蓝色呼吸。前进绿色全亮；左右转向显示对应方向累积流水，后退转向为红色，前进转向和原地转向为黄色，每 100 ms 增加一颗，全亮后全灭 100 ms 再循环，完整周期 0.7 秒；直线后退红色亮 1 秒、灭 1 秒。转向优先于前进和后退。

灯效使用有效速度指令，不使用码盘实测速度。沿用驱动的速度保活机制，停止发布 `/cmd_vel` 不等于进入停车灯效。灯珠暂按 0 在车辆左侧、5 在右侧排列；若方向相反，将 `ws2812_led.h` 中的 `WS2812_REVERSE_ORDER` 改为 `true`。主循环非阻塞更新，不修改串口协议、不使用 LEDC。接口、阈值和测试方法见 [灯带说明](docs/ws2812_led.md)。

## SLAM 建图与导航

`smart_tutorials` 默认启动串口底盘、机器人模型和 RViz，直接使用底盘转发的雷达与码盘里程计，不启动仿真或独立雷达驱动：

```bash
ros2 launch smart_tutorials slam.launch.py
# 建图期间另开终端保存地图，再退出建图入口：
ros2 run nav2_map_server map_saver_cli -f src/wp_smart/smart_tutorials/maps/map
ros2 launch smart_tutorials navigation.launch.py
```

导航默认加载 `smart_tutorials/maps/map.yaml`，可用 `map` 参数切换地图。已有底盘驱动时加 `start_base:=false`，无需图形界面时加 `use_rviz:=false`。建图和导航应分别运行。参数和完整流程见 [smart_tutorials/README.md](smart_tutorials/README.md)。

## 测试

```bash
colcon test --packages-select smart_bringup smart_description
colcon test-result --verbose
```

测试覆盖协议解析、扫描组装与 TF、码盘里程计、超时刹车、1 Hz 指令保持、重连清零、IMU 融合、OLED 数据处理及 udev 安装。串口联动测试使用伪终端；固件辅助逻辑测试会编译主机侧 C++ 程序。测试结果不能替代实车轮径、轮距、IMU 和电压标定。
