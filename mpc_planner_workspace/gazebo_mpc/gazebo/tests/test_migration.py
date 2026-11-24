"""ROS-free regression tests for scenario preservation and all-peer constraints."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import tempfile
import sys
import unittest
import xml.etree.ElementTree as E
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/"gazebo/scene")]
from scenarios import layout
from generate import generate
from dmpc_runtime.planner import LocalObjectMPC, PlannerConfig
from dmpc_runtime.prediction import ObjectState


class MigrationTests(unittest.TestCase):
    def test_original_two_robot_poses_and_environment_are_preserved(self):
        import os
        reference=Path(os.environ.get('ANTBOT_TEST_REFERENCE', str(ROOT)))
        if (reference/'isaac').exists():reference=reference/'isaac'
        original=reference/'simulation/helpers/scenario_config.py'
        spec=importlib.util.spec_from_file_location('original_scenario',original)
        module=importlib.util.module_from_spec(spec)
        import sys
        sys.modules[spec.name]=module;spec.loader.exec_module(module)
        for scenario in 'ABCD':
            old=module.scenario_layout(scenario)
            for count in (3,4):
                scene=layout(scenario,count)
                for i,robot in enumerate(('robot_a','robot_b')):
                    self.assertEqual(scene.robots[robot]['start'],old.starts[i])
                    self.assertEqual(scene.robots[robot]['goal'],old.goals[i])
                self.assertEqual(scene.bounds,old.bounds);self.assertEqual(scene.obstacles,old.obstacles)
                poses=[robot['start'] for robot in scene.robots.values()]
                for i,pose in enumerate(poses):
                    for other in poses[i+1:]:self.assertGreaterEqual(np.linalg.norm(np.array(pose[:2])-other[:2]),1.15)
    def test_generate_all_scenes_without_changing_reference(self):
        import os
        reference=Path(os.environ.get('ANTBOT_TEST_REFERENCE', str(ROOT)))
        if (reference/'isaac').exists():reference=reference/'isaac'
        for scenario in 'ABCD':
            for count in (3,4):
                with tempfile.TemporaryDirectory() as tmp:
                    scene=generate(reference,Path(tmp),scenario,count,check=True)
                    for robot in scene.robots:
                        root=E.parse(Path(tmp)/f'{robot}.urdf').getroot()
                        self.assertIsNone(root.find('ros2_control'))
                        self.assertIsNotNone(root.find("link[@name='base_link']/collision"))
                        self.assertEqual(root.find('gazebo/plugin/robot_namespace').text,robot)
                        self.assertEqual(root.find('gazebo/plugin/robot_count').text,str(count))
                        for mesh in root.findall('.//mesh'):self.assertTrue(Path(mesh.get('filename').removeprefix('file://')).exists())
    def test_all_peers_are_optimized_and_independently_validated(self):
        for count in (2,3):
            solver=LocalObjectMPC(PlannerConfig(peer_count=count,horizon=10,prediction_model='cv',solve_seconds=2.))
            peers=tuple(ObjectState(f'peer_{i}',0.,(10.,10.+i*3,0.),(0.,0.,0.)) for i in range(count))
            plan=solver.solve([0.,0.,0.,0.],[2.,0.,0.],peers,0.)
            self.assertEqual(len(plan.predictions),count);solver.validate(plan)
            # Replace only the final peer forecast with an immediate collision.
            changed=replace(plan.predictions[-1],positions=tuple((float(plan.states[0,k]),float(plan.states[1,k]),0.) for k in range(11)))
            bad=replace(plan,predictions=(*plan.predictions[:-1],changed))
            with self.assertRaisesRegex(ValueError,'object collision'):solver.validate(bad)
    def test_last_peer_time_zero_collision_cannot_be_ignored(self):
        solver=LocalObjectMPC(PlannerConfig(peer_count=3,horizon=5))
        peers=[ObjectState('b',0.,(10.,0.,0.),(0.,0.,0.)),ObjectState('c',0.,(0.,10.,0.),(0.,0.,0.)),
               ObjectState('d',0.,(.5,0.,0.),(0.,0.,0.))]
        with self.assertRaisesRegex(ValueError,'current object clearance'):solver.solve([0,0,0,0],[2,0,0],peers,0.)
        with self.assertRaisesRegex(ValueError,'distinct'):solver.solve([0,0,0,0],[2,0,0],peers[:2],0.)
        with self.assertRaisesRegex(ValueError,'distinct'):solver.solve([0,0,0,0],[2,0,0],[peers[0]]*3,0.)
    def test_peer_count_validation(self):
        for value in (0,4,True,2.5):
            with self.assertRaises(ValueError):PlannerConfig(peer_count=value)
        for scenario,count in [('X',3),('A',2),('A',5)]:
            with self.assertRaises(ValueError):layout(scenario,count)

if __name__=='__main__':unittest.main()
