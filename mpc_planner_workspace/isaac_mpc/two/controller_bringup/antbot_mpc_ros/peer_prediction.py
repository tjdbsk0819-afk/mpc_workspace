"""Mean and uncertainty for diagnostic paths; matches the shared C++ model."""
import math
from .cv_model import constant_velocity_prediction


def distribution(state, start_ns, dt_ns, steps, model, noise=.3, risk=.05):
    if model not in ('cv', 'cv_gaussian', 'gaussian_random_walk'):
        raise ValueError('Unknown peer prediction model')
    if start_ns < state.stamp_ns or dt_ns<=0 or not 2<=steps<=1000:
        raise ValueError('Invalid prediction time grid')
    if not math.isfinite(noise) or noise<=0 or not math.isfinite(risk) or not 0<risk<1:
        raise ValueError('Invalid Gaussian parameters')
    if model=='gaussian_random_walk':
        points=[(start_ns+k*dt_ns,state.x,state.y) for k in range(steps)]
    else:
        points=constant_velocity_prediction(state,start_ns,dt_ns,steps)
    result=[]
    for stamp,x,y in points:
        elapsed=(stamp-state.stamp_ns)/1e9
        sigma=0. if model=='cv' else noise*math.sqrt((dt_ns/1e9)*elapsed)
        result.append(dict(t=(stamp-start_ns)/1e9,x=x,y=y,sigma=sigma,
                           radius=math.sqrt(-2.*math.log(risk))*sigma))
    return result
