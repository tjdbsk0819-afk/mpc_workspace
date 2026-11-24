import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import yaml

from simulation.helpers.rpp_config import mpc_rpp_parameters

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT/'isaac/control_build/install/antbot_navigation/lib/antbot_navigation/mpc_rpp_controller'


@unittest.skipUnless(importlib.util.find_spec('rclpy') and BINARY.is_file(),
                     'Built ROS/Native RPP runtime is required')
class NativeRppShutdownTests(unittest.TestCase):
    def test_sigint_releases_plugin_publishers_before_unloading_the_library(self):
        template = yaml.safe_load((ROOT/'isaac/path_following_controller/config/isaac/rpp.yaml').read_text())
        execution = yaml.safe_load((ROOT/'isaac/path_following_controller/config/isaac/mpc_execution.yaml').read_text())['/**']['ros__parameters']
        with tempfile.TemporaryDirectory(prefix='dmpc-rpp-shutdown-') as temp:
            directory = Path(temp)
            params, output = directory/'params.yaml', directory/'process.log'
            params.write_text(yaml.safe_dump(mpc_rpp_parameters(template, execution, 'robot_a', ['robot_b'])))
            with output.open('w') as log:
                process = subprocess.Popen([str(BINARY), '--ros-args', '-r', '__ns:=/robot_a',
                                            '--params-file', str(params)],
                                           env=dict(os.environ, ROS_DOMAIN_ID='99'),
                                           stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic()+10.
                    while 'MPC-native RPP active' not in output.read_text():
                        self.assertIsNone(process.poll(), output.read_text())
                        self.assertLess(time.monotonic(), deadline, output.read_text())
                        time.sleep(.05)
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=10.)
                    # Previously both goals and motion could succeed, followed
                    # by SIGSEGV in LifecycleNode's weak publisher destruction.
                    self.assertEqual(process.returncode, 0, output.read_text())
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait()


if __name__ == '__main__':
    unittest.main()
