"""Body-frame omnidirectional kinematics shared by prediction and simulation."""
import numpy as np


def derivative(x, u):
    # Isaac uses the numeric state/scenario helpers without the MPC solver.
    # Load CasADi only when constructing symbolic optimization expressions.
    import casadi as ca
    c, s = ca.cos(x[2]), ca.sin(x[2])
    return ca.vertcat(c*u[0]-s*u[1], s*u[0]+c*u[1], u[2])


def step_symbolic(x, u, dt):
    k1 = derivative(x, u)
    k2 = derivative(x+dt*k1/2, u)
    k3 = derivative(x+dt*k2/2, u)
    k4 = derivative(x+dt*k3, u)
    return x+dt*(k1+2*k2+2*k3+k4)/6


def step(x, u, dt):
    x, u = np.asarray(x, float), np.asarray(u, float)
    def f(z):
        c, s = np.cos(z[2]), np.sin(z[2])
        return np.array([c*u[0]-s*u[1], s*u[0]+c*u[1], u[2]])
    k1 = f(x)
    k2 = f(x+dt*k1/2)
    k3 = f(x+dt*k2/2)
    k4 = f(x+dt*k3)
    return x+dt*(k1+2*k2+2*k3+k4)/6


def angle_error(a, b):
    return np.arctan2(np.sin(a-b), np.cos(a-b))


def segment_distance(a0, a1, b0, b1):
    """Minimum center distance between simultaneous linear segment motions."""
    r = np.asarray(a0)[:2]-np.asarray(b0)[:2]
    v = (np.asarray(a1)[:2]-np.asarray(a0)[:2])-(np.asarray(b1)[:2]-np.asarray(b0)[:2])
    alpha = np.clip(-r@v/(v@v), 0, 1) if v@v > 1e-15 else 0.
    return float(np.linalg.norm(r+alpha*v))
