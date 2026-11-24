import math
from pathlib import Path
import unittest
from reference import Trajectory, load_reference, tracking_command, motion_start_time


class ReferenceTest(unittest.TestCase):
    def test_heading_interpolates_across_pi(self):
        t = Trajectory([[0,0,0,math.pi-.1,1,0], [1,1,0,-math.pi+.1,1,0]])
        sample = t.at(.5)
        self.assertAlmostEqual(abs(sample[2]), math.pi)
        self.assertAlmostEqual(sample[0], .5)
        self.assertEqual(t.at(2)[3:], (0.,0.))

    def test_command_respects_acceleration_and_yaw_acceleration(self):
        v,vy,w = tracking_command((0,0,0),(5,5,1,1.5,.8),(0,0,0),.05)
        self.assertLessEqual(math.hypot(v,vy), .7*.05+1e-12)
        self.assertLessEqual(abs(w), .5*.05+1e-12)

    def test_followers_do_not_reverse(self):
        previous = (.5,0.,0.)
        for _ in range(40):
            command = tracking_command((0,0,0),(-1,0,0,-.5,0),previous,.05,(-.5,0))
            self.assertGreaterEqual(command[0],0.)
            self.assertLessEqual(math.hypot(command[0]-previous[0],command[1]-previous[1]),.7*.05+1e-12)
            previous = command
        self.assertAlmostEqual(previous[0],0.)
        vx,_,_ = tracking_command((0,0,math.pi),(-1,0,math.pi,1,0),(0,0,0),.05,(-1,0))
        self.assertGreater(vx,0.)

    def test_per_robot_initial_wait_removed(self):
        for delay in (5., 20.):
            rows = [[0,0,0,0,0,0], [delay-.1,0,0,0,0,0],
                    [delay,0,0,0,.1,0], [delay+1,1,0,0,1,0]]
            offset = motion_start_time(rows)
            self.assertEqual(offset, delay)
            self.assertAlmostEqual(Trajectory(rows).at(offset+.5)[0], .5)
        stationary = [[0,0,0,0,0,0], [1,0,0,0,0,0]]
        self.assertEqual(motion_start_time(stationary), 0)

    def test_A_only_purple_route_changed_to_lower_side(self):
        folder=Path(__file__).resolve().parents[1]/'references'
        data=load_reference(folder/'A.json','A');original=load_reference(folder/'A_recorded.json','A')
        for robot in ('robot_a','robot_b','robot_c'):
            self.assertEqual(data['robots'][robot],original['robots'][robot])
        rows=data['robots']['robot_d']
        self.assertTrue(all(-1.6-1e-9<=row[2]<=1e-9 for row in rows))
        self.assertEqual(rows[-1][1:3],[-6.,0.])
        self.assertEqual([r[0] for r in rows],[r[0] for r in original['robots']['robot_d']])
        self.assertTrue(all(0.<=row[4]<=1.5 and abs(row[5])<=.8 for row in rows))
        crows=data['robots']['robot_c'];c=Trajectory(crows);d=Trajectory(rows)
        co=motion_start_time(crows);do=motion_start_time(rows)
        minimum=min(math.hypot(c.at(i*.05+co)[0]-d.at(i*.05+do)[0],
                               c.at(i*.05+co)[1]-d.at(i*.05+do)[1]) for i in range(1700))
        self.assertGreater(minimum,2.5)

    def test_B_smooth_forward_crossing_routes(self):
        import itertools
        folder=Path(__file__).resolve().parents[1]/'references'
        data=load_reference(folder/'B.json','B')
        endpoints={'robot_a':((-3.,0.),(3.,0.)), 'robot_b':((0.,-3.),(0.,3.)),
                   'robot_c':((3.,0.),(-3.,0.)), 'robot_d':((0.,3.),(0.,-3.))}
        for robot,rows in data['robots'].items():
            start,goal=endpoints[robot];dx,dy=goal[0]-start[0],goal[1]-start[1]
            progress=[((row[1]-start[0])*dx+(row[2]-start[1])*dy)/36. for row in rows]
            self.assertTrue(all(b>=a-1e-12 for a,b in zip(progress,progress[1:])))
            self.assertTrue(all(-1e-9<=q<=1.+1e-9 for q in progress))
            self.assertEqual(tuple(rows[-1][1:3]),goal)
            self.assertEqual(rows[-1][4:],[0.,0.])
            self.assertLessEqual(max(abs(dx*(row[2]-start[1])-dy*(row[1]-start[0]))/6. for row in rows),1.+1e-9)
            self.assertTrue(all(0.<=row[4]<=1.5 and abs(row[5])<=.8 for row in rows))
        trajectories={r:Trajectory(rows) for r,rows in data['robots'].items()}
        offsets={r:motion_start_time(rows) for r,rows in data['robots'].items()}
        for a,b in itertools.combinations(trajectories,2):
            minimum=min(math.hypot(trajectories[a].at(i*.05+offsets[a])[0]-trajectories[b].at(i*.05+offsets[b])[0],
                                   trajectories[a].at(i*.05+offsets[a])[1]-trajectories[b].at(i*.05+offsets[b])[1]) for i in range(501))
            self.assertGreater(minimum,1.3)

    def test_same_epoch_and_scenario_validation(self):
        path = Path(__file__).resolve().parents[1]/'references/A.json'
        data = load_reference(path, 'A')
        self.assertEqual(set(data['robots']), {f'robot_{r}' for r in 'abcd'})
        with self.assertRaises(ValueError):
            load_reference(path, 'D')
        for robot, rows in data['robots'].items():
            self.assertAlmostEqual(rows[0][0], 0.)
            self.assertGreater(rows[-1][0], 70.)


if __name__ == '__main__':
    unittest.main()
