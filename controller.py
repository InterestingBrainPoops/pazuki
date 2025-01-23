import control
class Controller:
    def __init__(self, A, B, Q, R):
        self.K = control.lqr(A, B, Q, R)
    def forward(self, error):
        return self.K * error
    
class InnerOuter:

    def __init__(self, outer_controller, inner_controller):
        self.outer = outer_controller;
        self.inner = inner_controller;
    def compute_outer(self, error):
        return self.outer.forward(error)
    def compute_inner(self, error):
        return self.inner.forward(error)