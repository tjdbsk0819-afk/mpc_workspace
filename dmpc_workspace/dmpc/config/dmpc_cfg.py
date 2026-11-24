from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class DMPCBase:
    N: int          # horizon length
    Q: np.ndarray   # state-deviation cost
    R: np.ndarray   # control-effort cost
    H: np.ndarray   # terminal state-deviation cost
    term: bool      # terminal constraint
    
@dataclass(frozen=True)
class DecentralizedParams(DMPCBase):
    mode: str       # decentralized mode (Gauss-Seidel vs Jacobi)
    # Planar decentralized comparison uses the same CV + Gaussian as Gazebo.
    prediction_model: str = "cv_gaussian"
    prediction_noise: float = 0.3
    prediction_risk: float = 0.05
    
@dataclass(frozen=True)
class DistributedParams(DMPCBase):
    pass
