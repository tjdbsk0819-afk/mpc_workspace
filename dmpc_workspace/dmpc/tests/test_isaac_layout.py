import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("interface_preflight", ROOT/"isaac/simulation/check_ros_interfaces.py")
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class IsaacLayoutTests(unittest.TestCase):
    def test_launcher_mounts_repository_at_same_path_as_ros_build(self):
        # Capture Docker commands without starting containers or touching GPUs.
        with tempfile.TemporaryDirectory(prefix="dmpc-layout-") as temp:
            directory = Path(temp)
            capture = directory/"commands.jsonl"
            docker = directory/"docker"
            docker.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "with open(os.environ['DMPC_TEST_CAPTURE'], 'a') as stream:\n"
                " stream.write(json.dumps({'root':os.environ['ISAAC_MPC_ROOT'], 'args':sys.argv[1:]})+'\\n')\n")
            docker.chmod(0o755)
            env = dict(os.environ, PATH=f"{directory}:{os.environ['PATH']}", DMPC_TEST_CAPTURE=str(capture))
            subprocess.run(["bash", str(ROOT/"isaac/docker/run-two-antbots.sh"), "--run", "--headless"],
                           env=env, check=True, capture_output=True, text=True)
            calls = [json.loads(line) for line in capture.read_text().splitlines()]
        self.assertEqual(len(calls), 4)
        self.assertEqual(calls[0]['args'][0], 'ps')
        self.assertTrue(all(call["root"] == str(ROOT) for call in calls))
        self.assertEqual(calls[-1]["args"][-2:], ["/workspace/isaac/simulation/run_two_antbots.sh", "--headless"])
        compose = yaml.safe_load((ROOT/"isaac/docker/compose.two_antbots.yaml").read_text())
        service = compose["services"]["isaac-lab"]
        self.assertEqual(service["working_dir"], "/workspace/isaac")
        self.assertEqual(service["environment"]["ANTBOT_ISAAC_WORKSPACE"], "/workspace/isaac")
        mount = next(v for v in service["volumes"] if v["target"] == "/workspace")
        self.assertIn("ISAAC_MPC_ROOT", mount["source"])

    def make_messages(self, root):
        for package, names in {
            "antbot_interfaces": ("_joint_command_feedback.py", "_peer_state.py", "_mpc_plan.py"),
            "gaemi_interfaces": ("_depth_camera_array.py", "_mono_camera_array.py"),
        }.items():
            directory = root/f"message_build/install/{package}/lib/python3.12/site-packages/{package}/msg"
            directory.mkdir(parents=True)
            for name in ("__init__.py", *names):
                (directory/name).write_text("")

    def test_preflight_detects_broken_generated_message_links(self):
        with tempfile.TemporaryDirectory(prefix="dmpc-messages-") as temp:
            root = Path(temp)
            self.make_messages(root)
            preflight.check_generated_files(root)
            path = root/"message_build/install/antbot_interfaces/lib/python3.12/site-packages/antbot_interfaces/msg/_joint_command_feedback.py"
            path.unlink()
            path.symlink_to(root/"unmounted-build/_joint_command_feedback.py")
            with self.assertRaisesRegex(RuntimeError, "Missing/broken ROS message file"):
                preflight.check_generated_files(root)

    def test_existing_ros_build_links_resolve_under_repository_mount(self):
        package = ROOT/"isaac/message_build/install/antbot_interfaces/lib/python3.12/site-packages/antbot_interfaces"
        if not package.is_dir():
            self.skipTest("No generated ROS underlay in this checkout")
        links = [path for path in package.rglob("*") if path.is_symlink()]
        self.assertTrue(links)
        for path in links:
            target = path.readlink()
            # Rebase the container's /workspace mount to the host checkout.
            mapped = ROOT/target.relative_to("/workspace")
            self.assertTrue(mapped.is_file(), f"Unresolvable generated link: {path} -> {target}")
