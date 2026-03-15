# gi5651_can_odom

`gi5651_can_odom` covers both directions:
- `CAN -> odom/TXT`: read GI5651 frames from Linux `socketcan`, publish
  `nav_msgs/msg/Odometry`, and optionally write TXT.
- `TXT -> CAN`: replay `CANID#DATA` TXT lines back onto a Linux `socketcan` interface.

Default parameters are stored here:
- `CAN -> odom/TXT`: `config/can_2_txt.yaml`
- `TXT -> CAN`: `config/txt_2can.yaml`

## Build

```bash
cd /home/lzz/can_odom
source /opt/ros/<your_ros_distro>/setup.bash
colcon build --merge-install
source install/setup.bash
```

If you only want to build this package:

```bash
colcon build --merge-install --packages-select gi5651_can_odom
```

## Packages Required For CAN2USB

If you only want to bridge the vendor `CAN2USB` device into local `vcan0` and do not
run the ROS 2 node yet, install these system packages first:

```bash
sudo apt install -y python3 iproute2 kmod can-utils usbutils libusb-1.0-0 libudev1
```

These packages are used by:
- `python3`: runs `/home/lzz/can_odom/tools/usbcan_to_socketcan.py`
- `iproute2`: provides `ip link`
- `kmod`: provides `modprobe`
- `can-utils`: provides `candump`
- `usbutils`: provides `lsusb`
- `libusb-1.0-0` and `libudev1`: runtime dependencies of the vendor `libusbcan.so`

You also still need one vendor artifact:
- `libusbcan.so`
- `usbcan_libusb_x64_*.tar.bz2` or `usbcan_libusb_x86_*.tar.bz2`

The bridge script in this workspace can auto-discover or auto-extract those vendor files.

If you also want to run the ROS 2 `gi5651_can_odom` node, install ROS dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## CAN -> odom/TXT

Default launch:

```bash
ros2 launch gi5651_can_odom gi5651_can_odom.launch.py
```

Convenience wrapper from the workspace root:

```bash
/home/lzz/can_odom/run_gi5651_can_odom.sh
```

Example with overrides:

```bash
ros2 launch gi5651_can_odom gi5651_can_odom.launch.py \
  can:=can0 \
  topic_name:=/odom \
  frame_id:=odom \
  child_frame_id:=base_link \
  txt_path:=/tmp/odom_logs \
  txt_name:=auto \
  txt_name_format:=%y%m%d%H%M%S.txt \
  txt_is:=on \
  txt_format:=can \
  socket_timeout_sec:=0.2
```

Parameter meanings:
- `can`: SocketCAN interface name, for example `can0` or `vcan0`
- `topic_name`: odometry publish topic, default `/odom`
- `frame_id`: `Odometry.header.frame_id`, default `odom`
- `child_frame_id`: `Odometry.child_frame_id`, default `base_link`
- `txt_path`: TXT output directory, or an explicit `.txt` file path
- `txt_name`: TXT file name; `auto` means generate a timestamp file name at startup
- `txt_name_format`: `strftime` template used when `txt_name` is `auto`
- `txt_is`: TXT logging switch, `on` or `off`
- `txt_format`: `can` writes raw `CANID#DATA` lines; `odom_csv` writes published odometry CSV
- `socket_timeout_sec`: CAN socket receive timeout in seconds

Behavior notes:
- The launch file always loads defaults from `config/can_2_txt.yaml`. Command-line arguments
  only override the keys you pass.
- The node checks whether the configured CAN interface is up before opening the TXT file.
  If the interface is missing or down, it logs an error and exits immediately.
- With `txt_format:=can`, each line looks like `10B#D2042D0000000000`.
- With `txt_format:=odom_csv`, each line is one published odometry sample.

To use a custom YAML file instead of the default config:

```bash
ros2 launch gi5651_can_odom gi5651_can_odom.launch.py \
  params_file:=/home/lzz/my_can_odom.yaml
```

## TXT -> CAN

Default launch:

```bash
ros2 launch gi5651_can_odom txt_2can.launch.py
```

Example with overrides:

```bash
ros2 launch gi5651_can_odom txt_2can.launch.py \
  can:=vcan0 \
  txt_path:=/tmp/replay_can.txt \
  send_interval_sec:=0.01 \
  loop:=off
```

Parameter meanings:
- `can`: SocketCAN interface name used for replay, for example `can0` or `vcan0`
- `txt_path`: input TXT file path; must point to a `.txt` file
- `send_interval_sec`: fixed delay between frames; `0.0` means send as fast as possible
- `loop`: replay switch, `on` repeats from the beginning, `off` exits after one pass

TXT format requirements:
- One non-empty line represents one CAN frame.
- Valid line format is `CANID#DATA`.
- Standard ID uses 3 hex digits, for example `10B#D2042D0000000000`.
- Extended ID uses 8 hex digits.
- Payload must contain an even number of hex digits and at most 8 bytes.
- Empty lines and lines starting with `#` are ignored.

Notes:
- The launch file always loads defaults from `config/txt_2can.yaml`. Command-line arguments
  only override the keys you pass.
- `txt_path` must exist before startup, otherwise the node exits with a file-not-found error.
- When editing YAML, keep string switches such as `txt_is: "on"` and `loop: "off"` quoted.

## TXT -> GPS CSV

If you already have a raw CAN TXT log and only want the GPS points inside it, use:

```bash
python3 /home/lzz/can_odom/tools/extract_gps_from_can_txt.py \
  /tmp/replay_can.txt \
  -o /tmp/replay_gps.csv
```

If you omit `-o`, the CSV is written to stdout:

```bash
python3 /home/lzz/can_odom/tools/extract_gps_from_can_txt.py /tmp/replay_can.txt
```

This tool expects the same `CANID#DATA` input format used by `TXT -> CAN`.

Extraction rules:
- Empty lines and lines starting with `#` are ignored.
- Only CAN IDs `0x20B` and `0x21B` are treated as GPS position frames.
- The payload must contain at least 8 bytes.
- Bytes `0..3` are decoded as little-endian `int32 latitude * 1e-7`.
- Bytes `4..7` are decoded as little-endian `int32 longitude * 1e-7`.

CSV output columns:
- `entry_index`: 1-based index of extracted GPS entries
- `line_number`: original line number in the TXT file
- `can_id`: source CAN ID, formatted like `0x20B`
- `latitude_deg`: latitude in degrees
- `longitude_deg`: longitude in degrees
- `raw_payload`: original payload hex text

Failure behavior:
- Invalid `CANID#DATA` lines cause the tool to exit with an error.
- If no `0x20B` or `0x21B` frames are found, the tool exits with a non-zero status.

## 厂商 USB-CAN 桥接到 `candump`

有些 USB-CAN 设备不会在系统里生成原生 `can0`，只提供厂商自己的 `libusbcan.so`
用户态库。对这种设备，本工作区提供了
`/home/lzz/can_odom/tools/usbcan_to_socketcan.py`，可以把厂商库收到的 CAN 帧转发到
本地 `SocketCAN` 接口，例如 `vcan0`。

推荐直接使用一键脚本：

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

如果你想手动执行桥接，也可以这样：

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

python3 /home/lzz/can_odom/tools/usbcan_to_socketcan.py \
  --socketcan-if vcan0 \
  --device-type 4 \
  --device-index 0 \
  --channel 0 \
  --baud 500k
```

如果需要固定到某个厂商库，再额外加上：

```bash
python3 /home/lzz/can_odom/tools/usbcan_to_socketcan.py \
  --lib-path /path/to/libusbcan.so \
  --socketcan-if vcan0 \
  --device-type 4 \
  --device-index 0 \
  --channel 0 \
  --baud 500k
```

然后在另一个终端查看转发后的数据：

```bash
candump vcan0
```

`candump vcan0` 里看到的数据来源是：
- 真实 CAN 总线
- USB 转 CAN 设备
- 厂商 `libusbcan.so`
- 本地桥接脚本
- `vcan0`

因此，`vcan0` 中显示的是“从真实总线桥接过来的数据”，不是系统自己产生的数据。
