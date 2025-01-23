import numpy as np
from math import cos as c
from math import sin as s
# defining the non-linear eqs of motion for the craft
class SimpleRocket:
    
    def __init__(self, initial_condition, dry_mass, l, Jz, Isp, fuel_mass):
        self.dry_mass = dry_mass
        self.fuel_mass = fuel_mass
        self.l = l
        self.Jz = Jz
        self.state = initial_condition
        self.Isp = Isp
    def step(self, state, control_input, dt):
        out = np.zeros((1, 6))
        theta = state[4]
        u = state[0]
        v = state[1]
        T = control_input[0]
        alpha = control_input[1]
        ct = c(theta)
        st = s(theta)
        g = 9.81
        self.fuel_mass += self.calculate_mass_flow(T) * dt
        mass = self.dry_mass + self.fuel_mass
        out[0] = ct * u - st * v
        out[1] = st * u + ct * v
        out[2] = -g * ct + (1/mass) * -s(alpha) * T
        out[3] = -g * st + (1/mass) * c(alpha) * T
        out[4] = state[5]
        out[5] = (1/self.Jz) * (-self.l * T * s(alpha))
        return out
    def calculate_mass_flow(self, T):
        return -T / (self.Isp * 9.81)
    