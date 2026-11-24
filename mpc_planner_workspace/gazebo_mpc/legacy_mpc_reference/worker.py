"""Run IPOPT in a child process so a ROS watchdog can keep publishing stops."""
import multiprocessing as mp
import os
import time


def _serve(connection, cfg, robots, obstacles, bounds):
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    from .planner import Planner
    try:
        planner = Planner(cfg, robots, obstacles, bounds)
        # First solve loads the IPOPT plugin and must happen before the parent
        # starts enforcing per-command deadlines. No actuator command is sent.
        import numpy as np
        states = np.array([[-2., 0., 0.], [2., 0., 0.]])[:robots]
        peer = np.tile([2., 0., 0.], (cfg.horizon+1, 1)) if robots == 1 else None
        planner.solve(states, states, np.zeros((robots, 3)), peer=peer)
        planner._warm = None
        connection.send(('ready', None))
        while True:
            request = connection.recv()
            if request is None:
                break
            connection.send(('result', planner.solve(**request)))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send(('error', str(error)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class SolverWorker:
    def __init__(self, cfg, robots, obstacles=(), bounds=None):
        context = mp.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_serve, args=(child, cfg, robots, obstacles, bounds), daemon=True)
        self.process.start()
        child.close()
        self.ready, self.busy = False, False
        self.started = time.monotonic()

    def poll(self):
        if self.connection.poll():
            try:
                kind, data = self.connection.recv()
            except EOFError:
                return 'error', 'Solver process exited'
            if kind == 'ready':
                self.ready = True
            else:
                self.busy = False
            return kind, data
        if not self.process.is_alive():
            return 'error', 'Solver process exited'
        return None

    def submit(self, request):
        if not self.ready or self.busy:
            return False
        self.connection.send(request)
        self.started = time.monotonic()
        self.busy = True
        return True

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=1.)
        self.connection.close()
