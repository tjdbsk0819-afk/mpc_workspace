'''Antbot USD, articulation, actuator, and joint definitions.'''

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg


ANTBOT_USD_PATH = str(Path(__file__).resolve().parent / 'robot_usd' / 'antbot.usd')

ANTBOT_DEFAULT_ENV_NAMESPACE = '/World/envs/env_0'
ANTBOT_RUNTIME_ROBOT_PRIM_TEMPLATE = '{ENV_REGEX_NS}/Robot'
ANTBOT_RUNTIME_ARTICULATION_ROOT_TEMPLATE = '{ENV_REGEX_NS}/Robot/base_link'

ANTBOT_STEERING_JOINT_NAMES = (
    'steering_front_left_joint',
    'steering_front_right_joint',
    'steering_rear_left_joint',
    'steering_rear_right_joint',
)
ANTBOT_WHEEL_JOINT_NAMES = (
    'wheel_front_left_joint',
    'wheel_front_right_joint',
    'wheel_rear_left_joint',
    'wheel_rear_right_joint',
)
ANTBOT_JOINT_NAMES = ANTBOT_STEERING_JOINT_NAMES + ANTBOT_WHEEL_JOINT_NAMES

ANTBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ANTBOT_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            'steering_.*_joint': 0.0,
            'wheel_.*_joint': 0.0,
        },
        joint_vel={
            'steering_.*_joint': 0.0,
            'wheel_.*_joint': 0.0,
        },
    ),
    actuators={
        'steering': ImplicitActuatorCfg(
            joint_names_expr=['steering_.*_joint'],
            velocity_limit_sim=6.5,
            effort_limit_sim=1000.0,
            stiffness=321.0359,
            damping=0.128414,
        ),
        'wheel': ImplicitActuatorCfg(
            joint_names_expr=['wheel_.*_joint'],
            velocity_limit_sim=50.0,
            effort_limit_sim=200.0,
            # Velocity-driven wheels must not hold a position target.
            stiffness=0.0,
            damping=10.0,
        ),
    },
)


__all__ = [
    'ANTBOT_CFG',
    'ANTBOT_DEFAULT_ENV_NAMESPACE',
    'ANTBOT_JOINT_NAMES',
    'ANTBOT_RUNTIME_ARTICULATION_ROOT_TEMPLATE',
    'ANTBOT_RUNTIME_ROBOT_PRIM_TEMPLATE',
    'ANTBOT_STEERING_JOINT_NAMES',
    'ANTBOT_USD_PATH',
    'ANTBOT_WHEEL_JOINT_NAMES',
]
