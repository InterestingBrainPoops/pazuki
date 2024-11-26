import matplotlib.pyplot as plt
import numpy as np
from trajectory import Trajectory
dt = 0.01;
total_sim_time = 10;
time_steps = (int)(total_sim_time / dt);
t = 0;
goals = []
log = []
control_inputs = []
state = [0.0, 0.0]
goal = [10.0, 0.0]
K_p = 10.0
K_d = 3.0
g = 9.81
m = 1
traj = Trajectory([0, 10, 0, 10, 0, 10, 0, 10], 2)
while(t <= time_steps):
    log.append(np.copy(state))
    goal[0] , goal[1]= traj.sample(t * dt)
    print(goal)
    goals.append(np.copy(goal))
    position_error = goal[0] - state[0]
    u = max(0, position_error * K_p - state[1] * K_d + m * 9.81)
    control_inputs.append(u)
    state[0] += state[1] * dt;
    state[1] += (u / m) * dt - g * dt;
    t += 1;
log = np.array(log)
goals = np.array(goals)
control_inputs = np.array(control_inputs)
print(goals[:,1]) 


fig, ax = plt.subplots(2, sharex = True)

ax[0].plot(np.arange(0, time_steps) * dt, log[:-1,0], color = 'r', label = "current_height")
ax[0].plot(np.arange(0, time_steps) * dt, goals[:-1,0], color = 'g', label = "goal")
ax[0].legend()
ax[0].set(xlabel='Time(s)', ylabel='Height (m)')
ax[1].plot(np.arange(0, time_steps) * dt, control_inputs[:-1])
ax[1].set(xlabel='Time(s)', ylabel='Control Effort (N)')
plt.show()