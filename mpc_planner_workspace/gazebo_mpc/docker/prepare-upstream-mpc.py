#!/usr/bin/env python3
"""Select upstream ROS 2 templates only for the packages used by AntBot."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
packages = ['mpc_planner_types', 'mpc_planner_util', 'mpc_planner_solver',
            'mpc_planner_modules', 'mpc_planner']
for folder in [ROOT / 'mpc_planning' / p for p in packages] + [ROOT / 'planner_dependencies/ros_tools']:
    for active, template in [('CMakeLists.txt', 'CMakeLists2.txt'), ('package.xml', 'package2.xml')]:
        # The solver's shipped ROS 2 CMake is an obsolete FORCES-only build.
        if folder.name == 'mpc_planner_solver' and active == 'CMakeLists.txt':
            shutil.copyfile(ROOT / 'docker/upstream_solver_ros2.cmake', folder / active)
        else:
            shutil.copyfile(folder / template, folder / active)
    print(f'ROS 2: {folder.name}')
    # These templates install include/<package> but export include/<package> as
    # the search root; #include <package/header.h> needs the parent include root.
    cmake = folder / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace('$<INSTALL_INTERFACE:include/${PROJECT_NAME}>',
                                              '$<INSTALL_INTERFACE:include>'))

# ros_tools uses ament_index_cpp in paths.h but omits it in its ROS 2 template.
ros_tools = ROOT / 'planner_dependencies/ros_tools'
cmake = ros_tools / 'CMakeLists.txt'
cmake.write_text(cmake.read_text().replace('  rclcpp\n', '  rclcpp\n  ament_index_cpp\n', 1))
package = ros_tools / 'package.xml'
package.write_text(package.read_text().replace('<depend>rclcpp</depend>',
                  '<depend>rclcpp</depend>\n  <depend>ament_index_cpp</depend>', 1))

# Upstream calls these macros with stream expressions; the ROS 2 wrapper takes
# a string. Adapt logging in the dependency, leaving planner/solver math intact.
logging = ros_tools / 'include/ros_tools/logging.h'
text = logging.read_text()
for level in ('INFO', 'WARN'):
    old = f'#define LOG_{level}_THROTTLE(rate, ...) __RCLCPP_{level}_STREAM_THROTTLE(rate, __VA_ARGS__)'
    new = (f'#define LOG_{level}_THROTTLE(rate, ...) do {{ std::ostringstream log_stream; '
           f'log_stream << __VA_ARGS__; __RCLCPP_{level}_STREAM_THROTTLE(rate, log_stream.str()); }} while (0)')
    text = text.replace(old, new)
if '#include <sstream>' not in text:
    text = '#include <sstream>\n' + text
logging.write_text(text)
