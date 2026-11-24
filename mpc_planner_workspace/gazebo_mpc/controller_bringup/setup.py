from glob import glob
from setuptools import setup

setup(
    name='antbot_mpc_ros', version='0.1.0', packages=['antbot_mpc_ros'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/antbot_mpc_ros']),
        ('share/antbot_mpc_ros', ['package.xml']),
        ('share/antbot_mpc_ros/launch', glob('launch/*.launch.py')),
        ('share/antbot_mpc_ros/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AntBot project', maintainer_email='maintainer@example.com',
    description='AntBot GT/CV peer prediction and controller/legacy MPC adapters', license='MIT',
    entry_points={'console_scripts': [
        'mpc_node = antbot_mpc_ros.mpc_node:main',
        'kinematic_plant = antbot_mpc_ros.kinematic_plant:main',
        'gt_peer_predictor = antbot_mpc_ros.gt_peer_predictor:main',
        'peer_prediction_viewer = antbot_mpc_ros.peer_prediction_viewer:main',
        'mpc_rpp_adapter = antbot_mpc_ros.mpc_rpp_adapter:main',
    ]},
)
