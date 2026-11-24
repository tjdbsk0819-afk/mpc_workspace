import math
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from legacy_mpc_reference.trajectory import Prediction
from legacy_mpc_reference.ros_contract import uniform_prediction_times


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def stamp(value):
    nanoseconds = int(round(value*1e9))
    sec, ns = divmod(nanoseconds, 1_000_000_000)
    return Time(sec=sec, nanosec=ns)


def yaw(q):
    values = np.array([q.x, q.y, q.z, q.w], float)
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or abs(norm-1.) > .05:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = values/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def set_pose(pose, state):
    pose.position.x, pose.position.y = map(float, state[:2])
    pose.orientation.z = math.sin(float(state[2])/2)
    pose.orientation.w = math.cos(float(state[2])/2)


def twist(command):
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = map(float, command)
    return message


def prediction_message(prediction):
    message = Path()
    message.header.frame_id = prediction.frame
    message.header.stamp = stamp(prediction.stamp)
    for k, state in enumerate(prediction.states):
        pose = PoseStamped()
        pose.header.frame_id = prediction.frame
        pose.header.stamp = stamp(prediction.stamp+k*prediction.dt)
        set_pose(pose.pose, state)
        message.poses.append(pose)
    return message


def parse_prediction(message, frame):
    if message.header.frame_id != frame or not 2 <= len(message.poses) <= 256:
        raise ValueError('Wrong prediction frame or length')
    if any(p.header.frame_id != frame for p in message.poses):
        raise ValueError('Prediction pose frames disagree')
    epoch = seconds(message.header.stamp)
    dt = uniform_prediction_times([seconds(p.header.stamp) for p in message.poses], epoch)
    states = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in message.poses]
    # Unwrap before interpolating across the +/-pi boundary.
    states = np.asarray(states)
    states[:, 2] = np.unwrap(states[:, 2])
    return Prediction(epoch, dt, states, frame)
