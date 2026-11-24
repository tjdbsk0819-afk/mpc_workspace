"""Select an optimized in-place-turn prefix for Nav2's spatial FollowPath API."""
import math


def alignment_heading(trajectory, age, yaw, yaw_rate, horizon):
    """Use a fresh MPC heading ahead of execution, before an opposite turn."""
    end = min(age+horizon, trajectory.times[-1])
    for k, t in enumerate(trajectory.times):
        if age < t < end and trajectory.yaw_rates[k]*yaw_rate <= 0.:
            end = t
            break
    target = trajectory.sample(end).yaw
    error = math.atan2(math.sin(target-yaw), math.cos(target-yaw))
    return target if abs(error) > .005 and error*yaw_rate > 0. else None


def rotation_prefix_count(points, speed, yaw_rate, travel_limit=.004, minimum_angle=.006):
    """Return a prefix length; zero means the MPC supplied no usable turn prefix.

    points are (x, y, yaw) from the *optimized* MPC trajectory. No pose or heading
    is synthesized. Nav2 rotates to the last pose's orientation when its goal is
    already within xy_goal_tolerance. That expresses a timed MPC's initial turn
    through FollowPath without sending any direct rotation command.
    """
    if speed > .005 or abs(yaw_rate) <= .02 or len(points) < 2:
        return 0
    x, y, yaw = points[0]
    count = 0
    for k, (px, py, heading) in enumerate(points[1:], 1):
        if math.hypot(px-x, py-y) > travel_limit:
            break
        delta = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
        if delta*yaw_rate > 0. and abs(delta) >= minimum_angle:
            count = k+1
    return count
