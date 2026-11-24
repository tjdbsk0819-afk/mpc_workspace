"""Select acados in the container's copy of the workspace."""
from pathlib import Path

import yaml

workspace = Path('/mpc_workspace')
settings_path = workspace / 'src/mpc_planner/mpc_planner_jackal/config/settings.yaml'
settings = yaml.safe_load(settings_path.read_text())
settings['solver_settings']['solver'] = 'acados'
settings_path.write_text(yaml.safe_dump(settings, sort_keys=False))

jackal_cmake = workspace / 'src/mpc_planner/mpc_planner_jackal/CMakeLists.txt'
jackal_cmake.write_text(jackal_cmake.read_text().replace(
    'src/ros2_jackal_planner.cpp', 'src/ros2_planner.cpp'
))
