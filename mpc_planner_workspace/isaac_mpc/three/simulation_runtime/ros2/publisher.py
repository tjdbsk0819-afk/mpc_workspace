'''ROS 2 publishers for Antbot joint, camera, and LiDAR outputs.'''

from __future__ import annotations

import math
import sys

import numpy as np

from robot_model.sensors import CAMERA_RGB_NAMES, LIDAR_2D_NAMES


CAMERA_PUBLISH_PERIOD = 0.033
LIDAR_PUBLISH_PERIOD = 0.1
JOINT_STATE_PUBLISH_PERIOD = 0.02
LIDAR_2D_RANGE_MIN = 0.1
LIDAR_2D_RANGE_MAX = 30.0
LIDAR_3D_RANGE_MIN = 0.1
LIDAR_3D_RANGE_MAX = 100.0

DEPTH_TOPIC = '/sensor/camera/gemini336l/front_view/image'
FRONT_VIEW_TOPIC = '/sensor/camera/v4l2_driver/front_view/image'
REAR_VIEW_TOPIC = '/sensor/camera/v4l2_driver/rear_view/image'
TRAFFIC_LIGHT_TOPIC = '/sensor/camera/v4l2_driver/traffic_light_view/image'
LIDAR_2D_TOPICS = {
    'front': '/sensor/lidar/front_2d_scan',
    'back': '/sensor/lidar/back_2d_scan',
}
LIDAR_3D_TOPIC = '/sensor/lidar/scan_3d'
JOINT_STATE_TOPIC = '/joint_states'

CAMERA_FRAME_IDS = {
    'back': 'camera_mono_back_optical_link',
    'front': 'camera_mono_front_optical_link',
    'left': 'camera_mono_left_optical_link',
    'right': 'camera_mono_right_optical_link',
    'front_depth': 'camera_stereo_front_depth_optical_link',
}
LIDAR_2D_FRAME_IDS = {
    'front': 'lidar_2d_front_link',
    'back': 'lidar_2d_back_link',
}
LIDAR_3D_FRAME_ID = 'lidar_3d_link'


def _as_numpy(value) -> np.ndarray:
    '''Copy an Isaac Lab tensor or ProxyArray into contiguous CPU memory.'''

    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.ascontiguousarray(np.asarray(value))


def _stamp_header(header, simulation_time: float, frame_id: str) -> None:
    seconds = int(simulation_time)
    nanoseconds = int(round((simulation_time - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    header.stamp.sec = seconds
    header.stamp.nanosec = nanoseconds
    header.frame_id = frame_id


def _rotate_world_to_sensor(
    vectors_w: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    xyz = quaternion_xyzw[:3]
    w = quaternion_xyzw[3]
    cross = 2.0 * np.cross(xyz, vectors_w)
    return vectors_w - w * cross + np.cross(xyz, cross)


class AntbotPublisher:
    '''Publish all Antbot state and sensor outputs from one shared ROS node.'''

    def __init__(self, node, robot) -> None:
        from gaemi_interfaces.msg import DepthCameraArray, MonoCameraArray
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import (
            Image,
            JointState,
            LaserScan,
            PointCloud2,
            PointField,
        )

        self._node = node
        self._robot = robot
        self._image_type = Image
        self._depth_array_type = DepthCameraArray
        self._mono_array_type = MonoCameraArray
        self._laser_scan_type = LaserScan
        self._point_cloud_type = PointCloud2
        self._point_field_type = PointField
        self._joint_state_type = JointState

        self._next_joint_state_time = 0.0
        self._next_camera_time = 0.0
        self._next_lidar_2d_time = 0.0
        self._next_lidar_3d_time = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._depth_publisher = node.create_publisher(
            DepthCameraArray,
            DEPTH_TOPIC,
            sensor_qos,
        )
        self._front_view_publisher = node.create_publisher(
            MonoCameraArray,
            FRONT_VIEW_TOPIC,
            sensor_qos,
        )
        self._rear_view_publisher = node.create_publisher(
            MonoCameraArray,
            REAR_VIEW_TOPIC,
            sensor_qos,
        )
        self._traffic_light_publisher = node.create_publisher(
            MonoCameraArray,
            TRAFFIC_LIGHT_TOPIC,
            sensor_qos,
        )
        self._lidar_2d_publishers = {
            name: node.create_publisher(
                LaserScan,
                LIDAR_2D_TOPICS[name],
                sensor_qos,
            )
            for name in LIDAR_2D_NAMES
        }
        self._lidar_3d_publisher = node.create_publisher(
            PointCloud2,
            LIDAR_3D_TOPIC,
            sensor_qos,
        )
        self._joint_state_publisher = node.create_publisher(
            JointState,
            JOINT_STATE_TOPIC,
            sensor_qos,
        )

    @property
    def state_topic(self) -> str:
        return self._joint_state_publisher.topic_name

    @property
    def sensor_topics(self) -> tuple[str, ...]:
        return (
            self._depth_publisher.topic_name,
            self._front_view_publisher.topic_name,
            self._rear_view_publisher.topic_name,
            self._traffic_light_publisher.topic_name,
            *(publisher.topic_name for publisher in self._lidar_2d_publishers.values()),
            self._lidar_3d_publisher.topic_name,
        )

    def _publish_joint_state(self, simulation_time: float) -> bool:
        if simulation_time + 1.0e-9 < self._next_joint_state_time:
            return False
        self._next_joint_state_time = simulation_time + JOINT_STATE_PUBLISH_PERIOD
        message = self._joint_state_type()
        _stamp_header(message.header, simulation_time, '')
        message.name = list(self._robot.joint_names)
        message.position = _as_numpy(self._robot.data.joint_pos)[0].tolist()
        message.velocity = _as_numpy(self._robot.data.joint_vel)[0].tolist()
        message.effort = _as_numpy(self._robot.data.applied_torque)[0].tolist()
        self._joint_state_publisher.publish(message)
        return True

    def _rgb_image(self, scene, name: str, simulation_time: float):
        array = _as_numpy(scene[f'camera_{name}'].data.output['rgb'])
        if array.ndim == 4:
            array = array[0]
        if array.dtype != np.uint8:
            array = np.clip(array, 0, 255).astype(np.uint8)
        if array.ndim != 3 or array.shape[2] != 3:
            raise RuntimeError(
                f'Unexpected RGB shape for camera/{name}: {array.shape}'
            )
        message = self._image_type()
        _stamp_header(message.header, simulation_time, CAMERA_FRAME_IDS[name])
        message.height = int(array.shape[0])
        message.width = int(array.shape[1])
        message.encoding = 'rgb8'
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = array.tobytes()
        return message

    def _depth_image(self, scene, simulation_time: float):
        array = _as_numpy(
            scene['camera_front_depth'].data.output['distance_to_image_plane']
        )
        if array.ndim == 4:
            array = array[0]
        if array.ndim == 3 and array.shape[2] == 1:
            array = array[:, :, 0]
        array = np.ascontiguousarray(array, dtype=np.float32)
        if array.ndim != 2:
            raise RuntimeError(f'Unexpected depth shape: {array.shape}')
        message = self._image_type()
        _stamp_header(message.header, simulation_time, CAMERA_FRAME_IDS['front_depth'])
        message.height = int(array.shape[0])
        message.width = int(array.shape[1])
        message.encoding = '32FC1'
        message.is_bigendian = int(sys.byteorder == 'big')
        message.step = message.width * 4
        message.data = array.tobytes()
        return message

    def _publish_cameras(self, scene, simulation_time: float) -> bool:
        if simulation_time + 1.0e-9 < self._next_camera_time:
            return False
        self._next_camera_time = simulation_time + CAMERA_PUBLISH_PERIOD
        rgb = {
            name: self._rgb_image(scene, name, simulation_time)
            for name in CAMERA_RGB_NAMES
        }
        depth = self._depth_array_type()
        depth.color_images = [rgb['front']]
        depth.depth_images = [self._depth_image(scene, simulation_time)]
        self._depth_publisher.publish(depth)

        front = self._mono_array_type()
        front.images = [rgb['left'], rgb['front'], rgb['right']]
        self._front_view_publisher.publish(front)
        rear = self._mono_array_type()
        rear.images = [rgb['back']]
        self._rear_view_publisher.publish(rear)
        traffic = self._mono_array_type()
        traffic.images = [rgb['front']]
        self._traffic_light_publisher.publish(traffic)
        return True

    def _lidar_2d_message(self, sensor, name: str, simulation_time: float):
        hits_w = _as_numpy(sensor.data.ray_hits_w)[0]
        origin_w = _as_numpy(sensor.data.pos_w)[0]
        ranges = np.linalg.norm(hits_w - origin_w, axis=-1).astype(
            np.float32,
            copy=False,
        )
        valid = np.isfinite(ranges)
        valid &= ranges >= LIDAR_2D_RANGE_MIN
        valid &= ranges <= LIDAR_2D_RANGE_MAX
        ranges = np.where(valid, ranges, np.inf).astype(np.float32, copy=False)
        ray_count = int(ranges.size)
        angle_increment = 2.0 * math.pi / ray_count if ray_count else 0.0
        message = self._laser_scan_type()
        _stamp_header(message.header, simulation_time, LIDAR_2D_FRAME_IDS[name])
        message.angle_min = -math.pi
        message.angle_max = (
            message.angle_min + angle_increment * (ray_count - 1)
            if ray_count
            else message.angle_min
        )
        message.angle_increment = angle_increment
        message.time_increment = LIDAR_PUBLISH_PERIOD / ray_count if ray_count else 0.0
        message.scan_time = LIDAR_PUBLISH_PERIOD
        message.range_min = LIDAR_2D_RANGE_MIN
        message.range_max = LIDAR_2D_RANGE_MAX
        message.ranges = ranges.tolist()
        return message

    def _publish_lidar_2d(self, scene, simulation_time: float) -> bool:
        if simulation_time + 1.0e-9 < self._next_lidar_2d_time:
            return False
        self._next_lidar_2d_time = simulation_time + LIDAR_PUBLISH_PERIOD
        for name, publisher in self._lidar_2d_publishers.items():
            publisher.publish(
                self._lidar_2d_message(
                    scene[f'lidar_2d_{name}'],
                    name,
                    simulation_time,
                )
            )
        return True

    def _lidar_3d_message(self, sensor, simulation_time: float):
        hits_w = _as_numpy(sensor.data.ray_hits_w)[0]
        origin_w = _as_numpy(sensor.data.pos_w)[0]
        quaternion_xyzw = _as_numpy(sensor.data.quat_w)[0]
        finite = np.isfinite(hits_w).all(axis=-1)
        points_sensor = _rotate_world_to_sensor(
            hits_w[finite] - origin_w,
            quaternion_xyzw,
        )
        ranges = np.linalg.norm(points_sensor, axis=-1)
        valid = ranges >= LIDAR_3D_RANGE_MIN
        valid &= ranges <= LIDAR_3D_RANGE_MAX
        points_sensor = points_sensor[valid].astype(np.float32, copy=False)
        points = np.zeros((points_sensor.shape[0], 4), dtype=np.float32)
        points[:, :3] = points_sensor

        field = self._point_field_type
        message = self._point_cloud_type()
        _stamp_header(message.header, simulation_time, LIDAR_3D_FRAME_ID)
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = [
            field(name='x', offset=0, datatype=field.FLOAT32, count=1),
            field(name='y', offset=4, datatype=field.FLOAT32, count=1),
            field(name='z', offset=8, datatype=field.FLOAT32, count=1),
            field(name='intensity', offset=12, datatype=field.FLOAT32, count=1),
        ]
        message.is_bigendian = sys.byteorder == 'big'
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.data = points.tobytes()
        message.is_dense = True
        return message

    def _publish_lidar_3d(self, scene, simulation_time: float) -> bool:
        if simulation_time + 1.0e-9 < self._next_lidar_3d_time:
            return False
        self._next_lidar_3d_time = simulation_time + LIDAR_PUBLISH_PERIOD
        self._lidar_3d_publisher.publish(
            self._lidar_3d_message(scene['lidar_3d'], simulation_time)
        )
        return True

    def publish(self, scene, simulation_time: float) -> bool:
        '''Publish every joint, camera, and LiDAR message that is due.'''

        joint_state = self._publish_joint_state(simulation_time)
        camera = self._publish_cameras(scene, simulation_time)
        lidar_2d = self._publish_lidar_2d(scene, simulation_time)
        lidar_3d = self._publish_lidar_3d(scene, simulation_time)
        return joint_state or camera or lidar_2d or lidar_3d


__all__ = ['AntbotPublisher']
