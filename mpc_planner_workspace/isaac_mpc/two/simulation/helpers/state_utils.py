"""Array and frame conversions, independent of Isaac and ROS imports."""
import math
import numpy as np


def as_numpy(value):
    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.asarray(value)


def yaw_quaternion(yaw):
    """Isaac Lab 3.0 and ROS quaternion convention: x, y, z, w."""
    return (0., 0., math.sin(yaw/2), math.cos(yaw/2))


def world_to_body(vector, quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError('Invalid articulation quaternion')
    q = q/norm
    v = np.asarray(vector, float)
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError('Invalid articulation velocity')
    # Inverse unit-quaternion rotation, including roll and pitch.
    cross = 2*np.cross(q[:3], v)
    return v-q[3]*cross+np.cross(q[:3], cross)


def time_parts(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('Invalid simulation time')
    return divmod(int(round(seconds*1e9)), 1_000_000_000)


def root_state(robot):
    """Use Isaac Lab 3 root actor pose/velocity, not COM velocity."""
    pose = as_numpy(robot.data.root_link_pose_w)[0]
    velocity = as_numpy(robot.data.root_link_vel_w)[0]
    if pose.shape != (7,) or velocity.shape != (6,) or not np.isfinite(pose).all() or not np.isfinite(velocity).all():
        raise ValueError('Invalid articulation root state')
    return pose, world_to_body(velocity[:3], pose[3:]), world_to_body(velocity[3:], pose[3:])


def complete_joint_command(message, steering_names, wheel_names):
    """Reject partial/invalid commands before the upstream subscriber sees them."""
    if len(message.name) != len(set(message.name)):
        return False
    indices = {name: k for k, name in enumerate(message.name)}
    for names, values in [(steering_names, message.position), (wheel_names, message.velocity)]:
        for name in names:
            index = indices.get(name)
            if index is None or index >= len(values) or not math.isfinite(values[index]):
                return False
    return True


class CommandNodeAdapter:
    """Delegate Node API while validating and timing the existing subscription."""
    def __init__(self, node, steering_names, wheel_names, on_valid):
        self.node, self.steering_names, self.wheel_names = node, steering_names, wheel_names
        self.on_valid = on_valid

    def __getattr__(self, key):
        return getattr(self.node, key)

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        def validated(message):
            if complete_joint_command(message, self.steering_names, self.wheel_names):
                callback(message)
                self.on_valid()
        return self.node.create_subscription(message_type, topic, validated, qos, **kwargs)
