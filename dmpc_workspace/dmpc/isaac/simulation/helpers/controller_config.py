"""Configure two instances of existing AntBot controllers without editing sources."""
from copy import deepcopy
import xml.etree.ElementTree as ET


def namespaced_parameters(template, robot):
    config = deepcopy(template)
    manager = config['controller_manager']
    controller = config['antbot_swerve_controller']
    controller['ros__parameters']['cmd_vel_topic'] = f'/{robot}/cmd_vel'
    controller['ros__parameters']['enable_odom_tf'] = False
    # The C++ controller mixes flat get_parameter()/auto_declare() calls with
    # generate_parameter_library's antbot_swerve_controller.* parameters.
    # Feed both interfaces so topic routing and runtime updates stay consistent.
    params = controller['ros__parameters']
    params['antbot_swerve_controller'] = deepcopy({
        key: value for key, value in params.items()
        if key not in ('use_sim_time', 'antbot_swerve_controller')
    })
    return {f'/{robot}/controller_manager': manager,
            f'/{robot}/antbot_swerve_controller': controller}


def namespaced_control_description(xml, robot):
    root = ET.fromstring(xml)
    root.set('name', f'{robot}_control_only')
    control = root.find('ros2_control')
    if control is None:
        raise ValueError('Reference description has no ros2_control element')
    control.set('name', f'{robot}_isaac_system')
    found = set()
    for param in control.findall('hardware/param'):
        if param.get('name') == 'state_topic':
            param.text = f'/{robot}/joint_states'
            found.add('state_topic')
        elif param.get('name') == 'command_topic':
            param.text = f'/{robot}/joint_command'
            found.add('command_topic')
    if found != {'state_topic', 'command_topic'}:
        raise ValueError('Reference bridge does not expose both topic parameters')
    return ET.tostring(root, encoding='unicode')
