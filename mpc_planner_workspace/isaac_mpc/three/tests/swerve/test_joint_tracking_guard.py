"""Exercise the production C++ guard without starting ROS or Isaac."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class JointTrackingGuardTest(unittest.TestCase):
    def test_production_stop_and_restart_policy(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory(prefix='joint-tracking-guard-') as directory:
            binary = str(Path(directory)/'guard')
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(root/'swerve_controller/include'),
                            str(root/'tests/swerve/check_joint_tracking_guard.cpp'), '-o', binary], check=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    unittest.main()
