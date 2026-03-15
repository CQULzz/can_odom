#!/usr/bin/env python3

import argparse
import ctypes
import glob
import os
import platform
import pwd
import signal
import socket
import struct
import sys
import tarfile
import tempfile
from pathlib import Path


VCI_USBCAN2 = 4
STATUS_OK = 1
AUTO_LIB_PATH = "auto"

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000

BAUD_RAW_VALUES = {
    "1000k": 0x1400,
    "1m": 0x1400,
    "1000000": 0x1400,
    "800k": 0x1600,
    "800000": 0x1600,
    "666k": 0xB680,
    "500k": 0x1C00,
    "500000": 0x1C00,
    "400k": 0xFA80,
    "400000": 0xFA80,
    "250k": 0x1C01,
    "250000": 0x1C01,
    "200k": 0xFA81,
    "200000": 0xFA81,
    "125k": 0x1C03,
    "125000": 0x1C03,
    "100k": 0x1C04,
    "100000": 0x1C04,
    "80k": 0xFF83,
    "80000": 0xFF83,
    "50k": 0x1C09,
    "50000": 0x1C09,
    "40k": 0xFF87,
    "40000": 0xFF87,
    "20k": 0x1C18,
    "20000": 0x1C18,
    "10k": 0x1C31,
    "10000": 0x1C31,
}


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("TimeStamp", ctypes.c_uint32),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint32),
        ("AccMask", ctypes.c_uint32),
        ("Reserved", ctypes.c_uint32),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


STOP = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge a vendor USBCAN libusb device into a SocketCAN interface."
    )
    parser.add_argument(
        "--lib-path",
        default=AUTO_LIB_PATH,
        help="Path to libusbcan.so. Defaults to auto-discovery.",
    )
    parser.add_argument(
        "--socketcan-if",
        default="vcan0",
        help="SocketCAN interface to inject frames into, for example vcan0 or can0.",
    )
    parser.add_argument(
        "--device-type",
        type=int,
        default=VCI_USBCAN2,
        help="Vendor device type. USBCAN-II compatible devices typically use 4.",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="Vendor device index.",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=0,
        help="Vendor CAN channel index. 0 means CAN0, 1 means CAN1.",
    )
    parser.add_argument(
        "--baud",
        default="500k",
        help="CAN baud rate, for example 500k, 250k, 125k, 1m, or a raw timing value like 0x1C00.",
    )
    parser.add_argument(
        "--wait-ms",
        type=int,
        default=100,
        help="Receive wait time passed to VCI_Receive in milliseconds.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=256,
        help="Maximum number of frames fetched from the vendor library per poll.",
    )
    return parser.parse_args()


def get_user_home_candidates() -> list[Path]:
    homes: list[Path] = []
    seen: set[str] = set()

    for env_name in ("SUDO_USER", "USER"):
        username = os.environ.get(env_name)
        if not username:
            continue

        try:
            home = Path(pwd.getpwnam(username).pw_dir).expanduser()
        except KeyError:
            continue

        resolved = str(home)
        if resolved not in seen:
            homes.append(home)
            seen.add(resolved)

    home_env = os.environ.get("HOME")
    if home_env:
        home = Path(home_env).expanduser()
        resolved = str(home)
        if resolved not in seen:
            homes.append(home)
            seen.add(resolved)

    return homes


def iter_matching_paths(patterns: list[str]) -> list[Path]:
    matches: list[Path] = []
    seen: set[str] = set()

    for pattern in patterns:
        for raw_path in sorted(glob.glob(pattern)):
            if raw_path in seen:
                continue
            seen.add(raw_path)
            matches.append(Path(raw_path))

    return matches


def default_library_patterns() -> list[str]:
    temp_dir = Path(tempfile.gettempdir())
    patterns = [
        str(temp_dir / "usbcan_linux_demo_20260313" / "test" / "libusbcan.so"),
        str(temp_dir / "usbcan_libusb_x64_2018_10_15" / "test" / "libusbcan.so"),
        str(temp_dir / "usbcan_libusb_x86_2018_10_15" / "test" / "libusbcan.so"),
        str(temp_dir / "test" / "libusbcan.so"),
        str(temp_dir / "*" / "test" / "libusbcan.so"),
        str(Path("/var/tmp") / "*" / "test" / "libusbcan.so"),
    ]

    for home in get_user_home_candidates():
        patterns.extend(
            [
                str(home / "USBCAN" / "DRV" / "*" / "test" / "libusbcan.so"),
                str(home / "Downloads" / "*" / "test" / "libusbcan.so"),
                str(home / ".local" / "share" / "Trash" / "files" / "*" / "test" / "libusbcan.so"),
            ]
        )

    return patterns


def archive_name_patterns() -> list[str]:
    machine = platform.machine().lower()
    if machine in {"x86_64", "amd64"}:
        return ["usbcan_libusb_x64_*.tar.bz2", "usbcan_libusb_x86_*.tar.bz2"]
    return ["usbcan_libusb_x86_*.tar.bz2", "usbcan_libusb_x64_*.tar.bz2"]


def default_archive_patterns() -> list[str]:
    patterns: list[str] = []

    for home in get_user_home_candidates():
        for name in archive_name_patterns():
            patterns.extend(
                [
                    str(home / "USBCAN" / "DRV" / name),
                    str(home / "Downloads" / name),
                    str(home / ".local" / "share" / "Trash" / "files" / name),
                ]
            )

    return patterns


def extract_vendor_archive(archive_path: Path) -> Path:
    archive_name = archive_path.name
    extract_root = Path(tempfile.gettempdir()) / archive_name.removesuffix(".tar.bz2")
    lib_path = extract_root / "test" / "libusbcan.so"
    if lib_path.is_file():
        return lib_path

    extract_root.mkdir(parents=True, exist_ok=True)

    with tarfile.open(archive_path, "r:bz2") as tar:
        root_resolved = extract_root.resolve()
        for member in tar.getmembers():
            member_path = (extract_root / member.name).resolve()
            if os.path.commonpath((str(root_resolved), str(member_path))) != str(root_resolved):
                raise RuntimeError(f"Refusing to extract unsafe archive member: {member.name}")
        tar.extractall(extract_root)

    if not lib_path.is_file():
        raise FileNotFoundError(f"libusbcan.so was not found after extracting {archive_path}")

    return lib_path


def resolve_library_path(requested_path: str) -> Path:
    normalized = requested_path.strip()
    if normalized and normalized.lower() != AUTO_LIB_PATH:
        resolved = Path(normalized).expanduser()
        if not resolved.is_file():
            raise FileNotFoundError(f"libusbcan.so was not found: {resolved}")
        return resolved

    for candidate in iter_matching_paths(default_library_patterns()):
        if candidate.is_file():
            return candidate

    archive_errors: list[str] = []
    for archive_path in iter_matching_paths(default_archive_patterns()):
        try:
            lib_path = extract_vendor_archive(archive_path)
        except Exception as exc:
            archive_errors.append(f"{archive_path}: {exc}")
            continue

        print(f"Extracted vendor SDK from {archive_path} to {lib_path.parent}.", file=sys.stderr)
        return lib_path

    searched = [
        "/tmp/*/test/libusbcan.so",
        "/var/tmp/*/test/libusbcan.so",
        "~/USBCAN/DRV/usbcan_libusb_x64_*.tar.bz2",
        "~/Downloads/usbcan_libusb_x64_*.tar.bz2",
        "~/.local/share/Trash/files/usbcan_libusb_x64_*.tar.bz2",
    ]
    message = (
        "libusbcan.so was not found. "
        "Pass --lib-path explicitly, or place the vendor SDK archive in one of: "
        + ", ".join(searched)
    )
    if archive_errors:
        message += ". Archive errors: " + "; ".join(archive_errors)
    raise FileNotFoundError(message)


def parse_baud_raw(value: str) -> int:
    normalized = value.strip().lower()
    raw = BAUD_RAW_VALUES.get(normalized)
    if raw is not None:
        return raw

    try:
        return int(normalized, 0)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported baud value '{value}'. Use 500k/250k/125k/1m or a raw timing value like 0x1C00."
        ) from exc


def install_signal_handlers() -> None:
    def _handle_signal(signum, frame) -> None:  # type: ignore[unused-argument]
        global STOP
        STOP = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def load_library(path: Path) -> ctypes.CDLL:
    if not path.is_file():
        raise FileNotFoundError(f"libusbcan.so was not found: {path}")

    library = ctypes.CDLL(str(path))
    library.VCI_OpenDevice.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
    library.VCI_OpenDevice.restype = ctypes.c_uint32
    library.VCI_CloseDevice.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
    library.VCI_CloseDevice.restype = ctypes.c_uint32
    library.VCI_InitCAN.argtypes = [
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.POINTER(VCI_INIT_CONFIG),
    ]
    library.VCI_InitCAN.restype = ctypes.c_uint32
    library.VCI_StartCAN.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
    library.VCI_StartCAN.restype = ctypes.c_uint32
    library.VCI_Receive.argtypes = [
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.POINTER(VCI_CAN_OBJ),
        ctypes.c_uint32,
        ctypes.c_int,
    ]
    library.VCI_Receive.restype = ctypes.c_uint32
    return library


def open_socketcan(interface_name: str) -> socket.socket:
    can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        can_socket.bind((interface_name,))
    except OSError:
        can_socket.close()
        raise
    return can_socket


def vendor_frame_to_socketcan(frame: VCI_CAN_OBJ) -> bytes:
    can_id = int(frame.ID)
    if frame.ExternFlag:
        can_id |= CAN_EFF_FLAG
    if frame.RemoteFlag:
        can_id |= CAN_RTR_FLAG

    dlc = min(int(frame.DataLen), 8)
    payload = bytes(frame.Data[:dlc]).ljust(8, b"\x00")
    return struct.pack(CAN_FRAME_FORMAT, can_id, dlc, payload)


def main() -> int:
    args = parse_args()
    install_signal_handlers()

    try:
        baud_raw = parse_baud_raw(args.baud)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    try:
        resolved_lib_path = resolve_library_path(args.lib_path)
        library = load_library(resolved_lib_path)
    except Exception as exc:
        print(f"Failed to load vendor library: {exc}", file=sys.stderr)
        return 2

    print(f"Using vendor library: {resolved_lib_path}", file=sys.stderr)

    try:
        can_socket = open_socketcan(args.socketcan_if)
    except OSError as exc:
        print(
            f"Failed to open SocketCAN interface '{args.socketcan_if}': {exc}. "
            "Bring the interface up first.",
            file=sys.stderr,
        )
        return 2

    if library.VCI_OpenDevice(args.device_type, args.device_index, 0) != STATUS_OK:
        can_socket.close()
        print(
            "VCI_OpenDevice failed. If the device is present, try sudo or a udev rule for "
            "VID 0471 PID 1200.",
            file=sys.stderr,
        )
        return 1

    config = VCI_INIT_CONFIG(
        AccCode=0,
        AccMask=0xFFFFFFFF,
        Reserved=0,
        Filter=1,
        Timing0=baud_raw & 0xFF,
        Timing1=(baud_raw >> 8) & 0xFF,
        Mode=0,
    )

    if library.VCI_InitCAN(args.device_type, args.device_index, args.channel, ctypes.byref(config)) != STATUS_OK:
        library.VCI_CloseDevice(args.device_type, args.device_index)
        can_socket.close()
        print("VCI_InitCAN failed.", file=sys.stderr)
        return 1

    if library.VCI_StartCAN(args.device_type, args.device_index, args.channel) != STATUS_OK:
        library.VCI_CloseDevice(args.device_type, args.device_index)
        can_socket.close()
        print("VCI_StartCAN failed.", file=sys.stderr)
        return 1

    buffer_type = VCI_CAN_OBJ * args.batch_size
    buffer = buffer_type()
    forwarded_count = 0

    print(
        "Forwarding USBCAN device_type=%d device_index=%d channel=%d baud=%s -> %s"
        % (
            args.device_type,
            args.device_index,
            args.channel,
            args.baud,
            args.socketcan_if,
        )
    )
    print("Press Ctrl+C to stop.")

    try:
        while not STOP:
            received = int(
                library.VCI_Receive(
                    args.device_type,
                    args.device_index,
                    args.channel,
                    buffer,
                    args.batch_size,
                    args.wait_ms,
                )
            )
            if received <= 0:
                continue

            for index in range(received):
                can_socket.send(vendor_frame_to_socketcan(buffer[index]))
                forwarded_count += 1

            if forwarded_count == received or forwarded_count % 100 == 0:
                print("Forwarded %d CAN frame(s)." % forwarded_count)
    finally:
        library.VCI_CloseDevice(args.device_type, args.device_index)
        can_socket.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
