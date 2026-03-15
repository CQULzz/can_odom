import re
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_LINE_PATTERN = re.compile(r"^([0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})#([0-9A-Fa-f]*)$")


def switch_is_on(value: str) -> bool:
    return value.strip().lower() in {"1", "on", "true", "yes"}


def parse_can_line(line: str) -> Optional[bytes]:
    match = CAN_LINE_PATTERN.fullmatch(line.strip())
    if match is None:
        return None

    can_id_text, payload_text = match.groups()
    if len(payload_text) % 2 != 0:
        return None

    payload = bytes.fromhex(payload_text)
    if len(payload) > 8:
        return None

    can_id = int(can_id_text, 16)
    if len(can_id_text) == 8:
        if can_id > CAN_EFF_MASK:
            return None
        raw_can_id = can_id | CAN_EFF_FLAG
    else:
        if can_id > CAN_SFF_MASK:
            return None
        raw_can_id = can_id

    return struct.pack(
        CAN_FRAME_FORMAT,
        raw_can_id,
        len(payload),
        payload.ljust(8, b"\x00"),
    )


class Gi5651Txt2CanNode(Node):
    def __init__(self) -> None:
        super().__init__("gi5651_txt_2_can")

        self.declare_parameter("can", "can0")
        self.declare_parameter("txt_path", "/home/lzz/can_odom/odom_txt/replay_can.txt")
        self.declare_parameter("send_interval_sec", 0.01)
        self.declare_parameter("loop", "off")

        self.can_interface = str(self.get_parameter("can").value)
        self.txt_path = Path(str(self.get_parameter("txt_path").value)).expanduser()
        self.send_interval_sec = float(self.get_parameter("send_interval_sec").value)
        self.loop_enabled = switch_is_on(str(self.get_parameter("loop").value))

        if not self.can_interface:
            raise ValueError("Parameter 'can' must not be empty.")
        if self.send_interval_sec < 0.0:
            raise ValueError("Parameter 'send_interval_sec' must be non-negative.")
        if not self.txt_path.suffix.lower() == ".txt":
            raise ValueError("Parameter 'txt_path' must point to a .txt file.")
        if not self.txt_path.is_file():
            raise FileNotFoundError(f"TXT input file does not exist: {self.txt_path}")

        self._stop_event = threading.Event()
        self._replay_done = threading.Event()
        self._socket = self._open_can_socket()
        self._worker = threading.Thread(target=self._replay_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "Replaying %s to %s (loop=%s, send_interval_sec=%.3f)"
            % (self.txt_path, self.can_interface, self.loop_enabled, self.send_interval_sec)
        )

    def _open_can_socket(self) -> socket.socket:
        if not hasattr(socket, "AF_CAN") or not hasattr(socket, "CAN_RAW"):
            raise RuntimeError("This Python environment does not support SocketCAN.")

        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.bind((self.can_interface,))
        return can_socket

    def _replay_loop(self) -> None:
        sent_count = 0
        while rclpy.ok() and not self._stop_event.is_set():
            with self.txt_path.open("r", encoding="utf-8") as handle:
                for line_number, raw_line in enumerate(handle, start=1):
                    if self._stop_event.is_set():
                        self._replay_done.set()
                        return

                    line = raw_line.strip()
                    if not line or line.startswith("#"):
                        continue

                    frame = parse_can_line(line)
                    if frame is None:
                        self.get_logger().warning(
                            "Skipping invalid CAN text line %d: %s" % (line_number, raw_line.rstrip())
                        )
                        continue

                    try:
                        assert self._socket is not None
                        self._socket.send(frame)
                    except OSError as exc:
                        self.get_logger().error(f"CAN send failed: {exc}")
                        self._replay_done.set()
                        return

                    sent_count += 1
                    if sent_count == 1 or sent_count % 50 == 0:
                        self.get_logger().info("Sent %d CAN frames from TXT" % sent_count)

                    if self.send_interval_sec > 0.0:
                        time.sleep(self.send_interval_sec)

            if not self.loop_enabled:
                self.get_logger().info("TXT replay finished after sending %d CAN frames." % sent_count)
                self._replay_done.set()
                return

    def wait_until_finished(self) -> None:
        while not self._replay_done.is_set() and not self._stop_event.is_set():
            self._worker.join(timeout=0.2)

    def destroy_node(self) -> bool:
        self._stop_event.set()

        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=1.0)

        if getattr(self, "_socket", None) is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Gi5651Txt2CanNode()
    try:
        node.wait_until_finished()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
