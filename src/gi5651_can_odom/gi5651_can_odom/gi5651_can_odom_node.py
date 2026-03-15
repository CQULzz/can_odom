import fcntl
import math
import socket
import struct
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional
from typing import TextIO
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

ATTITUDE_FRAME_ID = 0x10B
POSITION_FRAME_IDS = {0x20B, 0x21B}
ALTITUDE_FRAME_IDS = {0x30B, 0x31B}
VELOCITY_FRAME_IDS = {0x40B, 0x41B}
GYRO_FRAME_ID = 0x60B
GYRO_ACCEL_FRAME_ID = 0x70B

TXT_FORMAT_CAN = "can"
TXT_FORMAT_ODOM_CSV = "odom_csv"
TXT_FORMAT_ALIASES = {
    "can": TXT_FORMAT_CAN,
    "can_signal": TXT_FORMAT_CAN,
    "raw_can": TXT_FORMAT_CAN,
    "odom": TXT_FORMAT_ODOM_CSV,
    "csv": TXT_FORMAT_ODOM_CSV,
    "odom_csv": TXT_FORMAT_ODOM_CSV,
}

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_READ_SIZE = 72
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
SIOCGIFFLAGS = 0x8913
IFF_UP = 0x1


class CanInterfaceNotReadyError(RuntimeError):
    pass


@dataclass
class CanFrame:
    stamp: datetime
    can_id: int
    data: bytes


class NavigationState:
    def __init__(self) -> None:
        self.heading_deg: Optional[float] = None
        self.pitch_deg: Optional[float] = None
        self.roll_deg: float = 0.0
        self.latitude_deg: Optional[float] = None
        self.longitude_deg: Optional[float] = None
        self.altitude_m: Optional[float] = None
        self.nav_flag: Optional[int] = None
        self.vel_enu: Optional[Tuple[float, float, float]] = None
        self.angular_velocity_body: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.origin_geodetic: Optional[Tuple[float, float, float]] = None
        self.origin_ecef: Optional[Tuple[float, float, float]] = None

    def has_pose(self) -> bool:
        return (
            self.heading_deg is not None
            and self.pitch_deg is not None
            and self.latitude_deg is not None
            and self.longitude_deg is not None
            and self.altitude_m is not None
        )

    def has_odometry(self) -> bool:
        return self.has_pose() and self.vel_enu is not None

    def reset_origin_if_needed(self) -> None:
        if self.origin_geodetic is not None:
            return
        if self.latitude_deg is None or self.longitude_deg is None or self.altitude_m is None:
            return
        self.origin_geodetic = (self.latitude_deg, self.longitude_deg, self.altitude_m)
        self.origin_ecef = geodetic_to_ecef(*self.origin_geodetic)


class Gi5651CanOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("gi5651_can_odom")

        self.declare_parameter("can", "can0")
        self.declare_parameter("topic_name", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("txt_path", "/home/lzz/can_odom/odom_txt")
        self.declare_parameter("txt_name", "auto")
        self.declare_parameter("txt_name_format", "%y%m%d%H%M%S.txt")
        self.declare_parameter("txt_is", "on")
        self.declare_parameter("txt_format", TXT_FORMAT_CAN)
        self.declare_parameter("socket_timeout_sec", 0.2)

        self.can_interface = str(self.get_parameter("can").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.txt_path_text = str(self.get_parameter("txt_path").value)
        self.txt_name = str(self.get_parameter("txt_name").value)
        self.txt_name_format = str(self.get_parameter("txt_name_format").value)
        self.txt_is_text = str(self.get_parameter("txt_is").value)
        self.txt_format = normalize_txt_format(str(self.get_parameter("txt_format").value))
        self.socket_timeout_sec = float(self.get_parameter("socket_timeout_sec").value)

        if not self.can_interface:
            raise ValueError("Parameter 'can' must not be empty.")
        if self.socket_timeout_sec <= 0.0:
            raise ValueError("Parameter 'socket_timeout_sec' must be greater than zero.")

        self._ensure_can_interface_is_up()

        self.publisher = self.create_publisher(Odometry, self.topic_name, 10)
        self.state = NavigationState()
        self._published_count = 0
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self._txt_file: Optional[TextIO] = None
        self._txt_file_path: Optional[Path] = None

        if switch_is_on(self.txt_is_text):
            self._txt_file_path = resolve_txt_output_path(
                self.txt_path_text, self.txt_name, self.txt_name_format
            )
            self._txt_file = open_txt_file(self._txt_file_path, self.txt_format)
            self.get_logger().info(
                f"TXT log enabled ({self.txt_format}): {self._txt_file_path}"
            )
        else:
            self.get_logger().info("TXT log disabled by parameter 'txt_is'.")

        self._socket = self._open_can_socket()
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

        self.get_logger().info(
            "Listening on %s and publishing odometry to %s"
            % (self.can_interface, self.topic_name)
        )

    def _ensure_can_interface_is_up(self) -> None:
        flags = get_interface_flags(self.can_interface)
        if flags is None:
            self.get_logger().error(
                "CAN interface '%s' was not found. Bring the CAN interface up before launching this node."
                % self.can_interface
            )
            raise CanInterfaceNotReadyError(self.can_interface)

        if not (flags & IFF_UP):
            self.get_logger().error(
                "CAN interface '%s' is down. Bring the CAN interface up before launching this node."
                % self.can_interface
            )
            raise CanInterfaceNotReadyError(self.can_interface)

    def _open_can_socket(self) -> socket.socket:
        if not hasattr(socket, "AF_CAN") or not hasattr(socket, "CAN_RAW"):
            raise RuntimeError("This Python environment does not support SocketCAN.")

        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.settimeout(self.socket_timeout_sec)
        can_socket.bind((self.can_interface,))
        return can_socket

    def _receive_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                assert self._socket is not None
                raw_frame = self._socket.recv(CAN_READ_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"CAN receive failed: {exc}")
                return

            frame = decode_can_frame(raw_frame)
            if frame is None:
                continue

            self._write_can_line(frame)

            try:
                self._handle_frame(frame)
            except Exception as exc:
                self.get_logger().error(f"Failed to decode CAN frame 0x{frame.can_id:X}: {exc}")

    def _handle_frame(self, frame: CanFrame) -> None:
        can_id = frame.can_id
        data = frame.data

        if can_id == ATTITUDE_FRAME_ID and len(data) >= 4:
            self.state.heading_deg = decode_u16(data, 0) * 1e-2
            self.state.pitch_deg = decode_s16(data, 2) * 1e-2
            if len(data) >= 6:
                self.state.roll_deg = decode_s16(data, 4) * 1e-2
        elif can_id in POSITION_FRAME_IDS and len(data) >= 8:
            self.state.latitude_deg = decode_s32(data, 0) * 1e-7
            self.state.longitude_deg = decode_s32(data, 4) * 1e-7
            self.state.reset_origin_if_needed()
        elif can_id in ALTITUDE_FRAME_IDS and len(data) >= 5:
            self.state.altitude_m = decode_s32(data, 0) * 1e-3
            self.state.nav_flag = data[4]
            self.state.reset_origin_if_needed()
        elif can_id in VELOCITY_FRAME_IDS and len(data) >= 6:
            east = decode_s16(data, 0) * 1e-2
            north = decode_s16(data, 2) * 1e-2
            up = decode_s16(data, 4) * 1e-2
            self.state.vel_enu = (east, north, up)
            if self.state.has_odometry():
                self._publish_odometry(frame)
        elif can_id == GYRO_FRAME_ID and len(data) >= 8:
            wx = math.radians(decode_s32(data, 0) * 1e-5)
            wy = math.radians(decode_s32(data, 4) * 1e-5)
            _, _, wz = self.state.angular_velocity_body
            self.state.angular_velocity_body = (wx, wy, wz)
        elif can_id == GYRO_ACCEL_FRAME_ID and len(data) >= 4:
            wx, wy, _ = self.state.angular_velocity_body
            wz = math.radians(decode_s32(data, 0) * 1e-5)
            self.state.angular_velocity_body = (wx, wy, wz)

    def _publish_odometry(self, frame: CanFrame) -> None:
        assert self.state.has_odometry()
        assert self.state.origin_geodetic is not None
        assert self.state.origin_ecef is not None
        assert self.state.latitude_deg is not None
        assert self.state.longitude_deg is not None
        assert self.state.altitude_m is not None
        assert self.state.pitch_deg is not None
        assert self.state.heading_deg is not None
        assert self.state.vel_enu is not None

        x, y, z = geodetic_to_enu(
            self.state.latitude_deg,
            self.state.longitude_deg,
            self.state.altitude_m,
            self.state.origin_geodetic,
            self.state.origin_ecef,
        )

        roll = math.radians(self.state.roll_deg)
        pitch = math.radians(self.state.pitch_deg)
        yaw = math.radians(90.0 - self.state.heading_deg)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        linear_body = rotate_world_to_body(self.state.vel_enu, roll, pitch, yaw)
        angular_body = self.state.angular_velocity_body
        stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.twist.twist.linear.x = linear_body[0]
        msg.twist.twist.linear.y = linear_body[1]
        msg.twist.twist.linear.z = linear_body[2]
        msg.twist.twist.angular.x = angular_body[0]
        msg.twist.twist.angular.y = angular_body[1]
        msg.twist.twist.angular.z = angular_body[2]

        self.publisher.publish(msg)
        self._published_count += 1
        self._write_odometry_line(msg, frame)

        if self._published_count == 1 or self._published_count % 50 == 0:
            yaw_deg = normalize_angle_deg(math.degrees(yaw))
            self.get_logger().info(
                "Published %d odom messages. Latest pos=(%.3f, %.3f, %.3f), "
                "ypr(deg)=(%.2f, %.2f, %.2f)"
                % (
                    self._published_count,
                    x,
                    y,
                    z,
                    yaw_deg,
                    self.state.pitch_deg,
                    self.state.roll_deg,
                )
            )

    def _write_odometry_line(self, msg: Odometry, frame: CanFrame) -> None:
        if self._txt_file is None or self.txt_format != TXT_FORMAT_ODOM_CSV:
            return

        line = (
            f"{frame.stamp.isoformat(timespec='milliseconds')},"
            f"{msg.header.stamp.sec},"
            f"{msg.header.stamp.nanosec},"
            f"{msg.header.frame_id},"
            f"{msg.child_frame_id},"
            f"{msg.pose.pose.position.x:.6f},"
            f"{msg.pose.pose.position.y:.6f},"
            f"{msg.pose.pose.position.z:.6f},"
            f"{msg.pose.pose.orientation.x:.9f},"
            f"{msg.pose.pose.orientation.y:.9f},"
            f"{msg.pose.pose.orientation.z:.9f},"
            f"{msg.pose.pose.orientation.w:.9f},"
            f"{msg.twist.twist.linear.x:.6f},"
            f"{msg.twist.twist.linear.y:.6f},"
            f"{msg.twist.twist.linear.z:.6f},"
            f"{msg.twist.twist.angular.x:.9f},"
            f"{msg.twist.twist.angular.y:.9f},"
            f"{msg.twist.twist.angular.z:.9f}\n"
        )
        self._txt_file.write(line)

    def _write_can_line(self, frame: CanFrame) -> None:
        if self._txt_file is None or self.txt_format != TXT_FORMAT_CAN:
            return

        self._txt_file.write(format_can_signal_line(frame))

    def destroy_node(self) -> bool:
        self._stop_event.set()

        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        if hasattr(self, "_receiver_thread") and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)

        if self._txt_file is not None:
            self._txt_file.close()
            self._txt_file = None

        return super().destroy_node()


def decode_can_frame(raw_frame: bytes) -> Optional[CanFrame]:
    if len(raw_frame) < CAN_FRAME_SIZE:
        return None

    raw_can_id, can_dlc, payload = struct.unpack(CAN_FRAME_FORMAT, raw_frame[:CAN_FRAME_SIZE])

    if raw_can_id & CAN_ERR_FLAG:
        return None
    if raw_can_id & CAN_RTR_FLAG:
        return None
    if can_dlc > 8:
        return None

    if raw_can_id & CAN_EFF_FLAG:
        can_id = raw_can_id & CAN_EFF_MASK
    else:
        can_id = raw_can_id & CAN_SFF_MASK

    return CanFrame(stamp=datetime.now(), can_id=can_id, data=payload[:can_dlc])


def get_interface_flags(interface_name: str) -> Optional[int]:
    if_name = interface_name.encode("utf-8")[:15]
    request = struct.pack("16sH", if_name, 0)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ioctl_socket:
            response = fcntl.ioctl(ioctl_socket.fileno(), SIOCGIFFLAGS, request)
    except OSError:
        return None

    return struct.unpack("16sH", response[:18])[1]


def resolve_txt_output_path(txt_path_text: str, txt_name: str, txt_name_format: str) -> Path:
    base_path = Path(txt_path_text).expanduser()
    if base_path.suffix.lower() == ".txt":
        return base_path

    if txt_name.strip() and txt_name.strip().lower() != "auto":
        file_name = txt_name.strip()
    else:
        file_name = datetime.now().strftime(txt_name_format)
    return base_path / file_name


def open_txt_file(path: Path, txt_format: str) -> TextIO:
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new_file = not path.exists() or path.stat().st_size == 0
    handle = path.open("a", encoding="utf-8", buffering=1)
    if is_new_file and txt_format == TXT_FORMAT_ODOM_CSV:
        handle.write(
            "receive_time,stamp_sec,stamp_nanosec,frame_id,child_frame_id,"
            "position_x,position_y,position_z,"
            "orientation_x,orientation_y,orientation_z,orientation_w,"
            "linear_x,linear_y,linear_z,"
            "angular_x,angular_y,angular_z\n"
        )
    return handle


def switch_is_on(value: str) -> bool:
    return value.strip().lower() in {"1", "on", "true", "yes"}


def normalize_txt_format(value: str) -> str:
    normalized = value.strip().lower()
    txt_format = TXT_FORMAT_ALIASES.get(normalized)
    if txt_format is None:
        valid_formats = ", ".join(sorted(set(TXT_FORMAT_ALIASES.values())))
        raise ValueError(f"Parameter 'txt_format' must be one of: {valid_formats}.")
    return txt_format


def format_can_signal_line(frame: CanFrame) -> str:
    can_id_width = 3 if frame.can_id <= CAN_SFF_MASK else 8
    can_id_text = f"{frame.can_id:0{can_id_width}X}"
    return f"{can_id_text}#{frame.data.hex().upper()}\n"


def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def decode_u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=False)


def decode_s16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], byteorder="little", signed=True)


def decode_s32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], byteorder="little", signed=True)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (radius + alt_m) * cos_lat * cos_lon
    y = (radius + alt_m) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return (x, y, z)


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_geodetic: Tuple[float, float, float],
    origin_ecef: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    dx = x - origin_ecef[0]
    dy = y - origin_ecef[1]
    dz = z - origin_ecef[2]

    lat0 = math.radians(origin_geodetic[0])
    lon0 = math.radians(origin_geodetic[1])
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return (east, north, up)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def rotate_world_to_body(
    vector_enu: Tuple[float, float, float], roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float]:
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)
    sy = math.sin(yaw)
    cy = math.cos(yaw)

    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )

    vx = (
        rotation[0][0] * vector_enu[0]
        + rotation[1][0] * vector_enu[1]
        + rotation[2][0] * vector_enu[2]
    )
    vy = (
        rotation[0][1] * vector_enu[0]
        + rotation[1][1] * vector_enu[1]
        + rotation[2][1] * vector_enu[2]
    )
    vz = (
        rotation[0][2] * vector_enu[0]
        + rotation[1][2] * vector_enu[1]
        + rotation[2][2] * vector_enu[2]
    )
    return (vx, vy, vz)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Gi5651CanOdomNode] = None
    try:
        node = Gi5651CanOdomNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except CanInterfaceNotReadyError:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
