#!/usr/bin/env python3
"""Fail before Kit startup when the shared ROS install cannot be imported."""
import argparse
from pathlib import Path


def check_generated_files(root):
    install = Path(root)/"message_build/install"
    expected = {
        "antbot_interfaces": ("_joint_command_feedback.py", "_peer_state.py", "_mpc_plan.py"),
        "gaemi_interfaces": ("_depth_camera_array.py", "_mono_camera_array.py"),
    }
    for package, filenames in expected.items():
        directories = list((install/package/"lib").glob(f"python*/site-packages/{package}/msg"))
        if len(directories) != 1:
            raise RuntimeError(f"Missing or ambiguous generated package: {install/package}. Run bash run_isaac.sh build.")
        directory = directories[0]
        for filename in ("__init__.py", *filenames):
            path = directory/filename
            if not path.is_file():
                target = f" -> {path.readlink()}" if path.is_symlink() else ""
                raise RuntimeError(
                    f"Missing/broken ROS message file: {path}{target}. "
                    "Build and Isaac containers must both mount the repository at /workspace "
                    "(ROS files at /workspace/isaac). Restart with bash run_isaac.sh sim --scenario B; "
                    "if generated files are missing, run bash run_isaac.sh build first.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    args = parser.parse_args()
    try:
        check_generated_files(args.project_root)
        from antbot_interfaces.msg import JointCommandFeedback, MpcPlan, PeerState
        from gaemi_interfaces.msg import DepthCameraArray, MonoCameraArray
        from rclpy.type_support import check_for_type_support
        for message in (JointCommandFeedback, MpcPlan, PeerState, DepthCameraArray, MonoCameraArray):
            check_for_type_support(message)
        print("[ROS2] Custom message imports and native type support ready", flush=True)
    except (ImportError, RuntimeError, OSError) as error:
        raise SystemExit(
            f"[ROS2] Interface preflight failed: {error}\n"
            "Verify /workspace/isaac/message_build is mounted at the same path used to build it.") from error


if __name__ == "__main__":
    main()
