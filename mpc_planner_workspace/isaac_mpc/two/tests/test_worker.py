import time
import unittest
import numpy as np
from legacy_mpc_reference.config import MPCConfig
from legacy_mpc_reference.worker import SolverWorker


class WorkerTests(unittest.TestCase):
    def test_process_solve_and_cleanup(self):
        cfg = MPCConfig(horizon=8, solver_max_seconds=2.)
        worker = SolverWorker(cfg, 2)
        try:
            limit = time.monotonic()+10
            while not worker.ready and time.monotonic() < limit:
                event = worker.poll()
                if event:
                    self.assertEqual(event[0], 'ready', event)
                time.sleep(.01)
            self.assertTrue(worker.ready)
            self.assertTrue(worker.submit({'states': np.array([[-2, -.8, 0], [2, .8, np.pi]]),
                                           'targets': np.array([[2, -.8, 0], [-2, .8, np.pi]]),
                                           'previous': np.zeros((2, 3))}))
            result = None
            limit = time.monotonic()+5
            while result is None and time.monotonic() < limit:
                result = worker.poll()
                time.sleep(.01)
            self.assertIsNotNone(result)
            self.assertEqual(result[0], 'result')
            self.assertTrue(result[1].success, result[1].status)
        finally:
            worker.close()
        self.assertFalse(worker.process.is_alive())


if __name__ == '__main__':
    unittest.main()
