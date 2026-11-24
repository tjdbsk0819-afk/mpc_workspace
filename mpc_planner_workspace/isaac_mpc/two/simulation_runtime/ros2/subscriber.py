'''ROS 2 input subscriptions for joint commands and optional sensor consumers.'''

from __future__ import annotations

import math

from robot_model.asset import (
    ANTBOT_STEERING_JOINT_NAMES,
    ANTBOT_WHEEL_JOINT_NAMES,
)


JOINT_COMMAND_TOPIC = '/joint_command'

SENSOR_TOPICS = {
    'camera_depth': '/sensor/camera/gemini336l/front_view/image',
    'camera_front': '/sensor/camera/v4l2_driver/front_view/image',
    'camera_rear': '/sensor/camera/v4l2_driver/rear_view/image',
    'camera_traffic_light': '/sensor/camera/v4l2_driver/traffic_light_view/image',
    'lidar_2d_front': '/sensor/lidar/front_2d_scan',
    'lidar_2d_back': '/sensor/lidar/back_2d_scan',
    'lidar_3d': '/sensor/lidar/scan_3d',
}


class JointCommandSubscriber:
    '''Receive JointState commands and apply them to CUDA-backed targets.'''

    def __init__(self, node, robot) -> None:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState

        self._node = node
        self._robot = robot
        self._has_new_target = False
        self._pending_command = None
        self.applied_command = None
        self._warned_invalid_command = False
        self._initialize_joint_targets()

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = node.create_subscription(
            JointState,
            JOINT_COMMAND_TOPIC,
            self._on_joint_command,
            command_qos,
        )

    @property
    def topic(self) -> str:
        return self._subscription.topic_name

    def _initialize_joint_targets(self) -> None:
        joint_indices = {
            name: index for index, name in enumerate(self._robot.joint_names)
        }
        try:
            self._steering_joint_ids = [
                joint_indices[name] for name in ANTBOT_STEERING_JOINT_NAMES
            ]
            self._wheel_joint_ids = [
                joint_indices[name] for name in ANTBOT_WHEEL_JOINT_NAMES
            ]
        except KeyError as error:
            raise RuntimeError(
                f'Antbot articulation is missing controlled joint '
                f'{error.args[0]!r}.'
            ) from error
        self._steering_targets = {
            name: 0.0 for name in ANTBOT_STEERING_JOINT_NAMES
        }
        self._wheel_targets = {
            name: 0.0 for name in ANTBOT_WHEEL_JOINT_NAMES
        }

    def _on_joint_command(self, message) -> None:
        message_indices = {name: index for index, name in enumerate(message.name)}
        updated = False
        for name in ANTBOT_STEERING_JOINT_NAMES:
            index = message_indices.get(name)
            if index is None or index >= len(message.position):
                continue
            value = float(message.position[index])
            if math.isfinite(value):
                self._steering_targets[name] = value
                updated = True
        for name in ANTBOT_WHEEL_JOINT_NAMES:
            index = message_indices.get(name)
            if index is None or index >= len(message.velocity):
                continue
            value = float(message.velocity[index])
            if math.isfinite(value):
                self._wheel_targets[name] = value
                updated = True
        if updated:
            self._has_new_target = True
            self._pending_command = message
        elif not self._warned_invalid_command:
            self._node.get_logger().warning(
                'Ignored /joint_command: steering needs finite position values '
                'and wheels need finite velocity values.'
            )
            self._warned_invalid_command = True

    def apply_latest(self) -> bool:
        '''Apply the newest command to CUDA without touching USD drive APIs.'''

        if not self._has_new_target:
            return False
        import torch

        steering = torch.tensor(
            [[self._steering_targets[name] for name in ANTBOT_STEERING_JOINT_NAMES]],
            dtype=torch.float32,
            device=self._robot.device,
        ).repeat(self._robot.num_instances, 1)
        wheel = torch.tensor(
            [[self._wheel_targets[name] for name in ANTBOT_WHEEL_JOINT_NAMES]],
            dtype=torch.float32,
            device=self._robot.device,
        ).repeat(self._robot.num_instances, 1)
        self._robot.set_joint_position_target(
            steering,
            joint_ids=self._steering_joint_ids,
        )
        self._robot.set_joint_velocity_target(
            wheel,
            joint_ids=self._wheel_joint_ids,
        )
        self._has_new_target = False
        self.applied_command = self._pending_command
        return True


class SensorSubscriber:
    '''Optionally consume all camera and LiDAR ROS topics.

    This class is intentionally not constructed by run_antbot.py. Isaac Sim
    publishes these topics for external perception nodes; construct this class
    only when the runner itself must consume sensor messages, for example for
    replay, monitoring, or a ROS-based sensor-fusion path.
    '''

    def __init__(self, node) -> None:
        from gaemi_interfaces.msg import DepthCameraArray, MonoCameraArray
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import LaserScan, PointCloud2

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        message_types = {
            'camera_depth': DepthCameraArray,
            'camera_front': MonoCameraArray,
            'camera_rear': MonoCameraArray,
            'camera_traffic_light': MonoCameraArray,
            'lidar_2d_front': LaserScan,
            'lidar_2d_back': LaserScan,
            'lidar_3d': PointCloud2,
        }
        self._latest_messages = {}
        self._subscriptions = {}
        for name, topic in SENSOR_TOPICS.items():
            self._subscriptions[name] = node.create_subscription(
                message_types[name],
                topic,
                lambda message, sensor_name=name: self._on_message(
                    sensor_name,
                    message,
                ),
                sensor_qos,
            )

    @property
    def topics(self) -> tuple[str, ...]:
        return tuple(subscription.topic_name for subscription in self._subscriptions.values())

    def _on_message(self, sensor_name: str, message) -> None:
        self._latest_messages[sensor_name] = message

    def latest(self, sensor_name: str):
        '''Return the newest message, or None before the first callback.'''

        if sensor_name not in SENSOR_TOPICS:
            raise KeyError(f'Unknown Antbot sensor: {sensor_name}')
        return self._latest_messages.get(sensor_name)


__all__ = ['JointCommandSubscriber', 'SensorSubscriber', 'SENSOR_TOPICS']
