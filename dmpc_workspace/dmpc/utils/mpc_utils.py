import casadi as ca
import numpy as np
from dmpc_runtime.prediction import PredictionConfig, observation_from_model_state, predict_object


def shift_pred(X):
    return np.hstack([X[:, 1:], X[:, -1:]])


def linear_warm_start(x0, xf, N):
    return np.hstack([x0 + (k / float(N)) * (xf - x0) for k in range(N + 1)])


def set_ipopt_solver(opti):
    opts = {
        "ipopt.print_level" : 0
    }
    opti.solver("ipopt", opts)


def sphere_target(x_self, x_other, radius):
    diff = x_other - x_self
    dist = ca.sqrt(ca.dot(diff, diff) + 1e-6)
    return x_other - radius * diff / dist


def sphere_target_np(x_self, x_other, radius):
    x_self = np.asarray(x_self, dtype=float).reshape(-1)
    x_other = np.asarray(x_other, dtype=float).reshape(-1)
    diff = x_other[0:3] - x_self[0:3]
    dist = np.sqrt(np.dot(diff, diff) + 1e-6)
    x_target = x_other.copy()
    x_target[0:3] = x_other[0:3] - radius * diff / dist
    return x_target.reshape(-1, 1)


def predict_from_current_states(states, model, now, dt, N, config):
    """Snapshot observations once per tick; never inspect another MPC solution."""
    prediction = PredictionConfig(config.prediction_model, config.prediction_noise, config.prediction_risk)
    if prediction.model != "cv" and model != "bicycle":
        raise ValueError("The upstream Gaussian radius is planar; use cv for 3D models")
    return [predict_object(observation_from_model_state(m, x, model, now), now, dt, N, prediction)
            for m, x in enumerate(states)]


def set_object_predictions(agents, predictions, m, d_min):
    other = [p for j, p in enumerate(predictions) if j != m]
    for i, prediction in enumerate(other):
        agents[m]["opti"].set_value(agents[m]["XYZ_others"][i], np.asarray(prediction.positions).T)
        agents[m]["opti"].set_value(agents[m]["SEP_others"][i],
                                    (d_min + np.asarray(prediction.radii)).reshape(1, -1))


def add_decentralized_collision_constraints(opti, X, XYZ_others, SEP_others, k):
    for XYZ_m, separation in zip(XYZ_others, SEP_others):
        opti.subject_to(ca.sumsqr(X[0:3, k] - XYZ_m[:, k]) >= separation[0, k] ** 2)


def add_distributed_collision_constraints(opti, X, M, nx, d_min, k, m=None):
    m_iter = range(M) if m is None else range(m, m + 1)
    for i in m_iter:
        for j in range(i + 1, M):
            xz = X[nx * i : nx * i + 3, k]
            xj = X[nx * j : nx * j + 3, k]
            opti.subject_to(ca.sumsqr(xz - xj) >= d_min ** 2)
