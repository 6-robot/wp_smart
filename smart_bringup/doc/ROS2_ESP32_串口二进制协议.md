# ESP32-S3 串口二进制协议

完整帧格式、字段和时序要求见 [串口协议](../../docs/serial_protocol.md)。对应固件为 [smart_esp32_serial](../../firmware/smart_esp32_serial/smart_esp32_serial.ino)。

- 通讯 UART1：GPIO17 TX、GPIO18 RX，`460800 8N1`，双方共地。
- 帧格式：`AA 55 TYPE LEN PAYLOAD CHECKSUM`，多字节字段使用 little-endian。
- 下行：`0x01` 速度命令、`0x02` 停止命令。
- 上行：`0x81` 底盘状态（53 字节）、`0x82` 原始 IMU（28 字节）、`0x83` 雷达扫描分片。
- 固件指令超时为 3 秒；ROS 默认每秒重发最后速度，主机超时刹车由 `enable_cmd_vel_timeout` 控制。

独立串口工具的用法见 [Python 串口检查工具](tools/python串口模拟器使用示例.txt)。
