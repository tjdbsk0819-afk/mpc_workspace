"""ROS message time conversion shared by the lightweight relay and planner."""
from builtin_interfaces.msg import Time


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def valid_stamp(stamp):
    return stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000


def ros_stamp(value):
    ns = round(value*1e9)
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)
