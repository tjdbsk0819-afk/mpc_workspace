"""Convert a four-robot rosbag into the same portable reference for both planners."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--scenario', choices=list('ABCD'), required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag), rosbag2_py.ConverterOptions('', ''))
    robots = {f'robot_{r}': [] for r in 'abcd'}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        robot = topic.strip('/').split('/')[0]
        if topic != f'/{robot}/odom' or robot not in robots:
            continue
        msg = deserialize_message(raw, Odometry)
        if msg.header.frame_id != 'world' or msg.child_frame_id != f'{robot}/base_link':
            raise ValueError('Unexpected odometry frame')
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        row = [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y, yaw, v.linear.x, v.angular.z]
        if not all(math.isfinite(value) for value in row):
            raise ValueError('Nonfinite odometry')
        rows = robots[robot]
        if rows and row[0] <= rows[-1][0]:
            raise ValueError('Odometry timestamps must increase')
        rows.append(row)
    if any(len(rows) < 2 for rows in robots.values()):
        raise ValueError('Missing robot odometry')
    # One epoch for all robots: do not independently shift each robot's timeline.
    epoch = min(rows[0][0] for rows in robots.values())
    for rows in robots.values():
        for row in rows:
            row[0] = round(row[0]-epoch, 9)
    reports = sorted(Path(args.bag).parent.glob(f'*_{args.scenario}_4.json'), key=lambda p: p.stat().st_mtime)
    source_result = json.loads(reports[-1].read_text())['result'] if reports else 'unknown'
    data = dict(version=1, scenario=args.scenario, frame='world', source_bag=Path(args.bag).name,
                source_result=source_result, source_epoch=epoch, columns=['t','x','y','yaw','v','w'], robots=robots)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, separators=(',', ':')))
    print(f'{target}: {source_result}; sha256={hashlib.sha256(target.read_bytes()).hexdigest()}')


if __name__ == '__main__':
    main()
