import numpy as np
from abc import abstractmethod, ABS
class CStateSpace:
    def __init__(self, A, B):
        self.A = A;
        self.B = B;

class DStateSpace:
    def __init__(self, Cmodel, dt):
        # forward euler for simplicity, replace with zero order hold approx later
        self.dt = dt
        self.A = np.eye(Cmodel.A.shape()[0], Cmodel.A.shape()[0]) + Cmodel.A * dt
        self.B = Cmodel.B * dt

class FullModel(ABS):
    # for a given state vector x, return xdot (derivative of state wrt time for a given state and input combination)
    @abstractmethod
    def forward(self, state, control_input):
        pass
