from glob import glob
from setuptools import setup

setup(
    name="antbot_dmpc_ros", version="0.1.0", packages=["antbot_dmpc_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/antbot_dmpc_ros"]),
        ("share/antbot_dmpc_ros", ["package.xml"]),
        ("share/antbot_dmpc_ros/launch", glob("launch/*.launch.py")),
        ("share/antbot_dmpc_ros/config", glob("config/*.yaml")),
    ], install_requires=["setuptools"], zip_safe=True,
    maintainer="DMPC workspace", maintainer_email="maintainer@example.com",
    description="IPOPT planning from local moving-object forecasts", license="MIT",
    entry_points={"console_scripts": [
        "dmpc_planner = dmpc_runtime.ros_node:main",
        "peer_state_relay = dmpc_runtime.state_relay:main",
    ]},
)
