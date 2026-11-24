"""Shared workspace settings and pre-Kit ROS 2/WebRTC environment setup.

This module is intentionally standard-library-only so it can be imported before
Isaac Sim (Kit) starts. ROS 2 is configured to use ``rmw_zenoh_cpp`` and the
session connects to the host Zenoh router on TCP port 7447.
"""

from __future__ import annotations

import errno
import fcntl
import os
import socket
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, TextIO

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ENV = WORKSPACE_ROOT / ".env"
PYTHON_VERSION_DIR = f"python{sys.version_info.major}.{sys.version_info.minor}"

ISAAC_ROS2_JAZZY_LIB = "/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib"
ISAAC_ROS2_JAZZY_PYTHON = "/isaac-sim/exts/isaacsim.ros2.core/jazzy/rclpy"
GAEMI_INTERFACES_PREFIX = WORKSPACE_ROOT / "message_build" / "install" / "gaemi_interfaces"
GAEMI_INTERFACES_PYTHON = GAEMI_INTERFACES_PREFIX / "lib" / PYTHON_VERSION_DIR / "site-packages"
GAEMI_INTERFACES_LIB = GAEMI_INTERFACES_PREFIX / "lib"

# Standard paths used by the ROS Jazzy rmw_zenoh_cpp deb package.
ROS_JAZZY_PREFIX = Path("/opt/ros/jazzy")
RMW_ZENOH_LIBRARY = ROS_JAZZY_PREFIX / "lib" / "librmw_zenoh_cpp.so"
RMW_ZENOH_LIB_DIRS = (
    ROS_JAZZY_PREFIX / "lib",
    ROS_JAZZY_PREFIX / "lib" / "x86_64-linux-gnu",
    ROS_JAZZY_PREFIX / "opt" / "zenoh_cpp_vendor" / "lib",
)
ZENOH_SESSION_CONFIG = WORKSPACE_ROOT / "simulation_runtime" / "ros2_config" / "zenoh_session.json5"


def read_workspace_setting(name: str, fallback: str) -> str:
    """Read one setting: process environment > workspace .env > fallback."""
    environment_value = os.environ.get(name)
    if environment_value:
        return environment_value
    if not WORKSPACE_ENV.is_file():
        return fallback
    for raw_line in WORKSPACE_ENV.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", maxsplit=1)
        if key.strip() == name:
            return value.strip().strip("\"'")
    return fallback


@dataclass(frozen=True)
class WorkspaceSettings:
    """Resolved workspace settings shared by the runner modules."""
    webrtc_public_ip: str
    webrtc_signal_port: int
    webrtc_stream_port: int
    web_viewer_port: int
    ros_domain_id: int
    physics_dt: float
    print_interval_steps: int


def load_workspace_settings() -> WorkspaceSettings:
    """Load and validate all workspace settings in one place."""
    ros_domain_id = int(read_workspace_setting("ROS_DOMAIN_ID", "30"))
    if not 0 <= ros_domain_id <= 232:
        raise SystemExit(f"ROS_DOMAIN_ID must be between 0 and 232, got {ros_domain_id}")
    physics_dt = float(read_workspace_setting("PHYSICS_DT", "0.01"))
    if physics_dt <= 0.0:
        raise SystemExit(f"PHYSICS_DT must be greater than zero, got {physics_dt}")
    print_interval_steps = int(read_workspace_setting("PRINT_INTERVAL_STEPS", "30"))
    if print_interval_steps < 1:
        raise SystemExit(f"PRINT_INTERVAL_STEPS must be at least 1, got {print_interval_steps}")
    return WorkspaceSettings(
        webrtc_public_ip=read_workspace_setting("ISAACSIM_HOST", "127.0.0.1"),
        webrtc_signal_port=int(read_workspace_setting("ISAACSIM_SIGNAL_PORT", "49100")),
        webrtc_stream_port=int(read_workspace_setting("ISAACSIM_STREAM_PORT", "47998")),
        web_viewer_port=int(read_workspace_setting("WEB_VIEWER_PORT", "8210")),
        ros_domain_id=ros_domain_id,
        physics_dt=physics_dt,
        print_interval_steps=print_interval_steps,
    )


class Ros2EnvironmentError(RuntimeError):
    """Raised when the ROS 2 runtime environment is not usable."""


class WebRtcEnvironmentError(RuntimeError):
    """Raised when the WebRTC stream cannot be started safely."""


def _check_port_available(port: int, socket_type: int, label: str) -> socket.socket:
    """Reserve one IPv4 port long enough to validate WebRTC startup."""
    probe = socket.socket(socket.AF_INET, socket_type)
    try:
        if socket_type == socket.SOCK_STREAM:
            # A stopped WebRTC server can leave accepted TCP connections in
            # TIME_WAIT. Match server restart semantics instead of treating
            # those reusable connections as a running Isaac instance.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("0.0.0.0", port))
        if socket_type == socket.SOCK_STREAM:
            # SO_REUSEPORT is deliberately not enabled: an active server must
            # still be rejected, including sockets bound with SO_REUSEADDR.
            probe.listen(1)
    except OSError as error:
        probe.close()
        if error.errno != errno.EADDRINUSE:
            raise WebRtcEnvironmentError(
                f"Cannot check WebRTC {label} port {port}: {error}"
            ) from error
        raise WebRtcEnvironmentError(
            f"WebRTC {label} port {port} cannot be reserved: {error}. "
            "Another server may own the port, or a previous connection may still "
            "be closing. Check the port with ss before restarting Isaac Sim."
        ) from error
    return probe


def ensure_webrtc_ports_available(settings: WorkspaceSettings) -> None:
    """Fail before Kit starts when another WebRTC server owns its ports."""
    probes: list[socket.socket] = []
    try:
        probes.append(
            _check_port_available(
                settings.webrtc_signal_port,
                socket.SOCK_STREAM,
                "signal TCP",
            )
        )
        probes.append(
            _check_port_available(
                settings.webrtc_stream_port,
                socket.SOCK_DGRAM,
                "stream UDP",
            )
        )
    finally:
        for probe in probes:
            probe.close()


@contextmanager
def reserve_webrtc_instance(settings: WorkspaceSettings) -> Iterator[TextIO]:
    """Hold a process lock and reject duplicate Antbot WebRTC servers."""
    lock_path = Path("/tmp") / (
        f"antbot-webrtc-{settings.webrtc_signal_port}-"
        f"{settings.webrtc_stream_port}.lock"
    )
    lock_file = lock_path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            lock_file.seek(0)
            owner_pid = lock_file.read().strip() or "unknown"
            raise WebRtcEnvironmentError(
                "Another run_antbot.py instance is already using this WebRTC "
                f"stream (PID {owner_pid})."
            ) from error

        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(str(os.getpid()))
        lock_file.flush()
        ensure_webrtc_ports_available(settings)
        yield lock_file
    finally:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            lock_file.close()


def configure_webrtc(settings: WorkspaceSettings) -> tuple[dict, str]:
    """Prepare the workspace WebRTC stream configuration."""
    os.environ["PUBLIC_IP"] = settings.webrtc_public_ip
    launcher_overrides = {
        "livestream": 1,
        "visualizer": ["kit"],
        "hide_ui": False,
        "multi_gpu": False,
    }
    kit_args = " ".join(
        (
            "--/exts/omni.kit.livestream.app/primaryStream/" f"publicIp={settings.webrtc_public_ip}",
            "--/exts/omni.kit.livestream.app/primaryStream/" f"signalPort={settings.webrtc_signal_port}",
            "--/exts/omni.kit.livestream.app/primaryStream/" f"streamPort={settings.webrtc_stream_port}",
            "--/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize=true",
        )
    )
    return launcher_overrides, kit_args


def _configure_zenoh_environment() -> None:
    """Select Zenoh and remove stale DDS profile variables."""
    if not RMW_ZENOH_LIBRARY.is_file():
        raise Ros2EnvironmentError(
            "rmw_zenoh_cpp is not installed in the Isaac Lab image. Install "
            "the ROS Jazzy package ros-jazzy-rmw-zenoh-cpp and rebuild the image "
            "before running simulation_runtime."
        )
    missing_paths = [
        str(path)
        for path in RMW_ZENOH_LIB_DIRS
        if path.is_dir() and str(path) not in os.environ.get("LD_LIBRARY_PATH", "").split(":")
    ]
    if missing_paths:
        export_paths = ":".join(missing_paths)
        raise Ros2EnvironmentError(
            "Zenoh runtime libraries are not in LD_LIBRARY_PATH. Run before "
            f"starting Isaac Sim: export LD_LIBRARY_PATH={export_paths}:$LD_LIBRARY_PATH"
        )
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    if ZENOH_SESSION_CONFIG.is_file():
        os.environ.setdefault("ZENOH_SESSION_CONFIG_URI", str(ZENOH_SESSION_CONFIG))


def configure_ros2_environment(
    settings: WorkspaceSettings, publishes_cameras: bool, enable_bridge: bool
) -> str:
    """Validate ROS paths, select Zenoh, and prepare the process environment."""
    ld_paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
    if ISAAC_ROS2_JAZZY_LIB not in ld_paths:
        raise Ros2EnvironmentError(
            "ROS 2 Jazzy libraries are not visible. Run: "
            f"export LD_LIBRARY_PATH={ISAAC_ROS2_JAZZY_LIB}:$LD_LIBRARY_PATH"
        )
    if ISAAC_ROS2_JAZZY_PYTHON not in sys.path:
        sys.path.append(ISAAC_ROS2_JAZZY_PYTHON)
    if publishes_cameras:
        if not GAEMI_INTERFACES_PYTHON.exists():
            raise Ros2EnvironmentError(
                "gaemi_interfaces is not built. Build ros_messages into message_build first."
            )
        if str(GAEMI_INTERFACES_LIB) not in ld_paths:
            raise Ros2EnvironmentError(
                "gaemi_interfaces libraries must be exported before Python starts: "
                f"export LD_LIBRARY_PATH={GAEMI_INTERFACES_LIB}:"
                f"{ISAAC_ROS2_JAZZY_LIB}:$LD_LIBRARY_PATH"
            )
        sys.path.insert(0, str(GAEMI_INTERFACES_PYTHON))
    os.environ["ROS_DOMAIN_ID"] = str(settings.ros_domain_id)
    os.environ.setdefault("ROS_DISTRO", "jazzy")
    # Isaac Sim is launched directly with /isaac-sim/python.sh, so it does not
    # inherit the environment that `source /opt/ros/jazzy/setup.bash` would set.
    # rmw_zenoh_cpp requires the ament index to locate its ROS dependencies.
    os.environ.setdefault("ROS_VERSION", "2")
    os.environ.setdefault("ROS_PYTHON_VERSION", "3")
    os.environ.setdefault("AMENT_PREFIX_PATH", str(ROS_JAZZY_PREFIX))
    os.environ.setdefault("CMAKE_PREFIX_PATH", str(ROS_JAZZY_PREFIX))
    _configure_zenoh_environment()
    if enable_bridge:
        return "--enable isaacsim.ros2.bridge"
    return ""
