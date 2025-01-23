import numpy as np
from ssmodel import FullModel

class hoverBlock(FullModel):
    def __init__(self, mass):
        self.mass = mass
        
    def forward(self, state, control_input):
        return  
