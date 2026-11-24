"""3/4 robots: state observations -> MPC/IPOPT -> native RPP -> Swerve."""
import os
import copy
import sys
from pathlib import Path
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parent))
from scenarios import layout, workspace
from simulation.helpers.controller_config import namespaced_parameters, namespaced_control_description
from simulation.helpers.rpp_config import mpc_rpp_parameters


def nodes(context):
    scenario=LaunchConfiguration('scenario').perform(context)
    count=int(LaunchConfiguration('robots').perform(context))
    backend=os.environ['ANTBOT_PLANNER']
    single_ego=LaunchConfiguration('single_ego').perform(context)
    reference=LaunchConfiguration('reference').perform(context)
    single_dir=Path(__file__).resolve().parent/'single_mpc'
    if single_ego:
        if count!=4 or single_ego not in [f'robot_{r}' for r in 'abcd']:
            raise ValueError('Single MPC mode requires four robots and a valid single_ego')
        sys.path.insert(0,str(single_dir))
        from reference import load_reference
        load_reference(reference,scenario)
    scene=layout(scenario,count);walls=workspace(scene)
    nav=Path(get_package_share_directory('antbot_navigation'))
    bridge=Path(get_package_share_directory('antbot_isaac_control'))
    control=yaml.safe_load((bridge/'config/swerve_controller.yaml').read_text())
    model=xacro.process_file(str(bridge/'urdf/antbot_control_only.urdf.xacro')).toxml()
    template=yaml.safe_load((nav/'config/isaac/rpp.yaml').read_text())
    execution=yaml.safe_load((nav/'config/isaac/mpc_execution.yaml').read_text())['/**']['ros__parameters']
    execution.update(walls)
    enabled=LaunchConfiguration('enabled').perform(context).lower()=='true'
    if single_ego: enabled=False  # Follower enables ego only after all four controllers are active.
    actions=[]
    output=Path('/tmp/antbot_gazebo')/backend;output.mkdir(parents=True,exist_ok=True)
    if backend=='dmpc':
        bringup=Path(get_package_share_directory('antbot_dmpc_ros'))
        planner_file=bringup/'config/dmpc.yaml'
        planner=yaml.safe_load(planner_file.read_text())['/**']['ros__parameters']
        execution.update({k:v for k,v in planner.items() if k.startswith('peer_prediction_')})
        execution['plan_timeout_enabled']=planner.get('plan_timeout_enabled',True)
        execution.update(max_speed=planner['max_speed'],max_yaw_rate=planner['max_yaw_rate'],
                         max_acceleration=planner['max_acceleration'],max_yaw_acceleration=planner['max_yaw_acceleration'],
                         minimum_separation=planner['separation'],state_timeout=planner['state_timeout'],plan_timeout=planner['plan_timeout'])
    else:
        bringup=Path(get_package_share_directory('antbot_mpc_ros'))
        planner_share=Path(get_package_share_directory('mpc_planner_antbot'))
        prediction=yaml.safe_load((Path(os.environ['ANTBOT_REFERENCE'])/'mpc_planning/mpc_planner_antbot/config/peer_prediction.yaml').read_text())
        # The existing two-robot launch's prediction file is a ROS parameter file.
        prediction=prediction.get('/**',prediction).get('ros__parameters',prediction)
        execution.update({k:v for k,v in prediction.items() if k.startswith('peer_prediction_')})
        settings=yaml.safe_load((planner_share/'config/settings.yaml').read_text())
        execution.update(max_speed=settings['antbot_limits']['speed'],
                         max_acceleration=settings['antbot_limits']['acceleration'],
                         max_yaw_rate=settings['antbot_limits']['yaw_rate'])
    # Keep Nav2's desired speed consistent with the selected planner limit.
    template['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=execution['max_speed']
    actions.append(LogInfo(msg=f"Gazebo {backend.upper()}: scenario={scenario}, robots={count}, "
        f"max_speed={execution['max_speed']} m/s, max_acceleration={execution['max_acceleration']} m/s^2"))
    def save(values,prefix):
        with tempfile.NamedTemporaryFile(mode='w',dir=output,prefix=prefix,suffix='.yaml',delete=False) as stream:
            yaml.safe_dump(values,stream);return stream.name
    for ego,config in scene.robots.items():
        peers=[peer for peer in scene.robots if peer!=ego]
        description=namespaced_control_description(model,ego)
        params=save(namespaced_parameters(control,ego),ego+'_control_')
        actions += [
            Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=ego,name='control_description',
                 parameters=[{'robot_description':description,'use_sim_time':True,'frame_prefix':f'{ego}/control_model/'}],
                 remappings=[('/tf',f'/{ego}/control_model/tf'),('/tf_static',f'/{ego}/control_model/tf_static')]),
            Node(package='controller_manager',executable='ros2_control_node',namespace=ego,
                 parameters=[params,{'use_sim_time':True}],remappings=[('robot_description',f'/{ego}/robot_description'),
                     ('~/robot_description',f'/{ego}/robot_description'),('odom',f'/{ego}/wheel_odom')],output='screen'),
            Node(package='controller_manager',executable='spawner',namespace=ego,
                 arguments=['antbot_swerve_controller','--controller-manager',f'/{ego}/controller_manager',
                            '--controller-manager-timeout','60','--param-file',params],output='screen')]
        if single_ego and ego!=single_ego:
            continue
        for peer in peers:
            if backend=='dmpc':
                actions.append(Node(package='antbot_dmpc_ros',executable='peer_state_relay',namespace=ego,name='state_'+peer,
                    parameters=[{'use_sim_time':True,'ego_robot':ego,'peer_robot':peer,'state_timeout':planner['state_timeout']}],
                    remappings=[('peer_state',f'peers/{peer}/state')],output='screen'))
            else:
                actions.append(Node(package='antbot_mpc_ros',executable='gt_peer_predictor',namespace=ego,name='state_'+peer,
                    parameters=[str(bringup/'config/gt_peer_cv.yaml'),prediction,
                                {'use_sim_time':True,'ego_robot':ego,'peer_robot':peer,'peer_odom_topic':f'/{peer}/odom'}],
                    remappings=[(f'/{ego}/peer_state',f'/{ego}/peers/{peer}/state'),
                                (f'/{ego}/peer_prediction',f'/{ego}/peers/{peer}/prediction')],output='screen'))
        goal=list(config['goal'])
        if backend=='dmpc':
            actions.append(Node(package='antbot_dmpc_ros',executable='dmpc_planner',namespace=ego,
                parameters=[str(planner_file),walls,{'use_sim_time':True,'ego_robot':ego,'peer_robot':peers[0],
                                                   'peer_robots':peers,'goal':goal,'allow_reverse':ego=='robot_a'}],output='screen'))
        else:
            actions.append(Node(package='mpc_planner_antbot',executable='antbot_planner',namespace=ego,
                parameters=[str(planner_share/'config/recovery.yaml'),prediction,walls,
                            {'use_sim_time':True,'ego_robot':ego,'peer_robots':peers,'goal':goal[:2],
                             'goal_yaw':goal[2],'recovery_enabled':True}],output='screen'))
        robot_template=copy.deepcopy(template)
        robot_execution=dict(execution)
        robot_execution['allow_reverse']=backend=='dmpc' and ego=='robot_a'
        if robot_execution['allow_reverse']:
            follow=robot_template['controller_server']['ros__parameters']['FollowPath']
            follow['allow_reversing']=True
            follow['use_rotate_to_heading']=False
        params=save(mpc_rpp_parameters(robot_template,robot_execution,ego,peers,enabled),ego+'_rpp_')
        actions.append(Node(package='antbot_navigation',executable='mpc_rpp_controller',namespace=ego,parameters=[params],output='screen'))
    if single_ego:
        actions.append(ExecuteProcess(cmd=['python3',str(single_dir/'follower.py'),
            '--reference',reference,'--scenario',scenario,'--ego',single_ego],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scenario',default_value='B',choices=list('ABCD')),
        DeclareLaunchArgument('robots',default_value='3',choices=['3','4']),
        DeclareLaunchArgument('single_ego',default_value=''),
        DeclareLaunchArgument('reference',default_value=''),
        DeclareLaunchArgument('enabled',default_value='true'),OpaqueFunction(function=nodes)])
