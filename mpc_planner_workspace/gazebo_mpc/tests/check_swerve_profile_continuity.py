"""Compile/run the real C++ profile regression in the MPC ROS image after build."""
from pathlib import Path
import shlex
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
flags=root/'control_build/build/antbot_swerve_controller/CMakeFiles/antbot_swerve_controller.dir/flags.make'
includes=next(line.split('=',1)[1].strip() for line in flags.read_text().splitlines() if line.startswith('CXX_INCLUDES ='))
with tempfile.TemporaryDirectory(prefix='swerve-continuity-') as output:
    binary=str(Path(output)/'check')
    subprocess.run(['c++','-std=c++17',*shlex.split(includes),str(root/'tests/check_swerve_profile_continuity.cpp'),
                    '-L'+str(root/'control_build/install/antbot_swerve_controller/lib'),'-lantbot_swerve_controller',
                    '-L/opt/ros/jazzy/lib','-lrclcpp','-lrcutils','-o',binary],check=True)
    subprocess.run([binary],check=True)
