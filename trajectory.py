class Trajectory:
    def __init__(self, points, dt):
        self.dt = dt
        self.points = points
    def sample(self, t):
        point = t / self.dt
        point_time = int(point) * self.dt
        behind = self.points[int(point)]
        ahead = self.points[int(point) + 1]
        goal = (behind + (ahead-behind) * ((t - point_time) / self.dt))
        deriv = (ahead - behind) / self.dt
        return (goal, deriv)
    
# def TrapezoidalProfile:
#     def __init__(self, initial, max_accel , final, dt):
