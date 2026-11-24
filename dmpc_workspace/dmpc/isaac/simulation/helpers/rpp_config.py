"""Scope the standalone RPP and its child costmap to one Isaac robot."""
from copy import deepcopy


def namespaced_rpp_parameters(template, robot):
    config = deepcopy(template)
    controller = config['controller_server']['ros__parameters']
    controller['odom_topic'] = f'/{robot}/odom'
    controller['speed_limit_topic'] = f'/{robot}/speed_limit'
    costmap = config['local_costmap']['local_costmap']['ros__parameters']
    costmap['robot_base_frame'] = f'{robot}/base_link'
    return {
        f'/{robot}/controller_server': config['controller_server'],
        f'/{robot}/local_costmap/local_costmap': config['local_costmap']['local_costmap'],
    }


def mpc_rpp_parameters(template, execution, robot, peers, enabled=True):
    """Native RPP owns MPC input, time sampling, safety and cmd_vel output."""
    params = namespaced_rpp_parameters(template, robot)
    original = params.pop(f'/{robot}/controller_server')['ros__parameters']
    native = {key: deepcopy(original[key]) for key in ('use_sim_time', 'FollowPath', 'goal_checker')}
    native.update(deepcopy(execution))
    native.update(ego_robot=robot, peer_robots=list(peers), enabled=enabled,
                  controller_frequency=execution['output_frequency'])
    controller = {'ros__parameters': native}
    params[f'/{robot}/mpc_rpp_controller'] = controller
    return params
