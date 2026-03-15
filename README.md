# xmt.can_odom

这是 `gi5651_can_odom` 的 ROS 2 工作区，包源码在 `src/gi5651_can_odom/`。

## 功能概述

这个包当前可以实现三类功能：
- `CAN -> odom/TXT`：从 CAN 总线读取 GI5651 数据，解析后发布 `nav_msgs/msg/Odometry`
- `CAN -> TXT`：把收到的 CAN 帧或 odom 结果保存成文本文件
- `TXT -> CAN`：把文本中的 `CANID#DATA` 帧重新回放到 CAN 总线

此外，针对不提供原生 `SocketCAN can0` 的厂商 USB 转 CAN 设备，本工作区还提供：
- `USB-CAN -> vcan0` 桥接
- 配合 `candump vcan0` 观察真实总线数据

## 构建

```bash
cd /home/lzz/can_odom
source /opt/ros/<your_ros_distro>/setup.bash
colcon build --merge-install
source install/setup.bash
```

如果只想编这个包：

```bash
colcon build --merge-install --packages-select gi5651_can_odom
```

## CAN2USB 需要哪些包

如果你只是想把这块厂商 `CAN2USB` 设备桥接到本机 `vcan0`，不跑 ROS 2 节点，
系统里至少需要这些包：

```bash
sudo apt install -y python3 iproute2 kmod can-utils usbutils libusb-1.0-0 libudev1
```

这些包分别对应：
- `python3`：运行 `tools/usbcan_to_socketcan.py`
- `iproute2`：提供 `ip link`
- `kmod`：提供 `modprobe`
- `can-utils`：提供 `candump`
- `usbutils`：提供 `lsusb`
- `libusb-1.0-0`、`libudev1`：厂商 `libusbcan.so` 的运行时依赖

除了上面的系统包，还需要厂商提供的以下任意一种：
- `libusbcan.so`
- `usbcan_libusb_x64_*.tar.bz2` 或 `usbcan_libusb_x86_*.tar.bz2`

本工作区的桥接脚本会自动尝试查找或解包这些厂商文件。

如果你还要运行 ROS 2 里的 `gi5651_can_odom` 节点，建议先安装 `rosdep`，再在工作区根目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 直接运行

CAN 转 odom/TXT：

```bash
ros2 launch gi5651_can_odom gi5651_can_odom.launch.py \
  can:=vcan0 \
  topic_name:=/odom \
  txt_is:=on \
  txt_format:=can
```

TXT 转 CAN：

```bash
ros2 launch gi5651_can_odom txt_2can.launch.py \
  can:=can0 \
  txt_path:=/tmp/replay_can.txt \
  send_interval_sec:=0.01 \
  loop:=off
```

## USB 转 CAN 设备说明

你当前这块 USB 转 CAN 设备走的是厂商 `libusbcan.so` 方案，不是原生 `SocketCAN` 网卡。

这意味着：
- 装好厂商驱动后，也不会自动在 `ifconfig` 或 `ip link` 里出现 `can0`
- `lsusb` 能看到设备是正常现象
- 想让 `candump` 收到数据，需要先桥接到一个本地 `SocketCAN` 接口，例如 `vcan0`

## 桥接到 `vcan0`

本工作区提供了两个工具：
- `tools/usbcan_to_socketcan.py`：把厂商库收到的 CAN 帧转发到本地 `SocketCAN`
- `tools/start_usbcan_vcan_bridge.sh`：一键起 `vcan0` 并启动桥接

推荐直接用一键脚本：

```bash
sudo /home/lzz/can_odom/tools/start_usbcan_vcan_bridge.sh
```

脚本会优先自动查找已有的 `libusbcan.so`，也会尝试从这些位置自动解包厂商 SDK：
- `/tmp/*/test/libusbcan.so`
- `~/USBCAN/DRV/usbcan_libusb_x64_*.tar.bz2`
- `~/Downloads/usbcan_libusb_x64_*.tar.bz2`
- `~/.local/share/Trash/files/usbcan_libusb_x64_*.tar.bz2`

如果你已经知道 `.so` 的准确路径，也可以显式指定：

```bash
sudo LIB_PATH=/path/to/libusbcan.so /home/lzz/can_odom/tools/start_usbcan_vcan_bridge.sh
```

如果设备接在第二路 CAN，或者波特率不是 `500k`，可以这样：

```bash
sudo CHANNEL=1 BAUD=250k /home/lzz/can_odom/tools/start_usbcan_vcan_bridge.sh
```

脚本会自动做这些事：
- 加载 `vcan` 内核模块
- 创建 `vcan0`
- 执行 `ip link set vcan0 up`
- 自动定位或解包厂商 `libusbcan.so`
- 把收到的真实 CAN 帧转发到 `vcan0`

## 用 `candump` 看数据

启动桥接后，另开一个终端执行：

```bash
candump vcan0
```

`candump vcan0` 里看到的数据来源是：
- 真实 CAN 总线上的数据
- 经 USB 转 CAN 设备接收
- 由厂商 `libusbcan.so` 读出
- 再由桥接脚本写入本机 `vcan0`

也就是说，`vcan0` 里显示的是“桥接后的总线数据”，不是系统自己凭空生成的数据。

## 什么时候会看不到数据

如果 `candump vcan0` 没有输出，优先检查：
- USB 转 CAN 设备是否已被系统识别：`lsusb | grep 0471:1200`
- 桥接脚本是否在运行
- `CHANNEL` 是否选对
- `BAUD` 是否和真实总线一致
- 真实 CAN 总线上是否本来就在发数据

更详细的参数说明见 `src/gi5651_can_odom/README.md`。
