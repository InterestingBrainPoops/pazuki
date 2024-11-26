import matplotlib.pyplot as plt
import control
import numpy as np
from trajectory import Trajectory
import math

A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
Q = np.diag(np.array([100, 1]))
R = np.diag(np.array([.1]))
def create_traj(func, dt, domain):
        points = []
        for t in range(0, int((domain[1] - domain[0]) / dt)):
            points.append(func(t * dt + domain[0]))
        return Trajectory(points, dt)
K, S, E = control.lqr(A, B, Q, R)
print(K)

dt = 0.01;
total_sim_time = 20;
time_steps = (int)(total_sim_time / dt);
t = 0;
log = []
control_inputs = []
state = np.transpose(np.array([0., 0.]))

goal = np.transpose(np.array([10., 0.]))
g = np.transpose(np.array([0, -9.81]))
def sin_path(t):
    return 5 *math.sin(t)
def step_path(t):
    if(math.floor(t) % 5 == 0):
        return 0
    return 2 
traj = create_traj(sin_path, dt, (0, total_sim_time + 10))
goals = []
while(t <= time_steps):
    goal[0] , goal[1]= traj.sample(t * dt)
    print(goal)
    goals.append(np.copy(goal))
    error = goal - state;
    u = np.matmul(K, error);
    u[0] = u[0] + 9.81
    u[0] = min(max(u[0], 0), 30)
    
    log.append(np.copy(state))
    control_inputs.append(np.copy(u))
    state += (np.matmul(A, state) + np.matmul(B, u)) * dt + g * dt;
    t += 1;
log = np.array(log)
control_inputs = np.array(control_inputs)
goals = np.array(goals)
print(goals[:, 0]) 


fig, ax = plt.subplots(2, sharex = True)

ax[0].plot(np.arange(0, time_steps) * dt, log[:-1,0], color = 'g', label = 'current height')
ax[0].plot(np.arange(0, time_steps) * dt, goals[:-1,0], label = 'goal')
ax[0].legend()
ax[0].set(xlabel='Time(s)', ylabel='Height (m)')
ax[1].plot(np.arange(0, time_steps) * dt, control_inputs[:-1,0])
ax[1].set(xlabel='Time(s)', ylabel='Control Effort (N)')
plt.show()
print(B.shape)

def simple_sim(A, B, K, goal, initial, labels, total_time = 5.0, dt = 0.01):
    state = np.copy(initial)
    time_steps = (int)(total_time / dt);
    t = 0;
    while(t <= time_steps):
        # error = goal - state;
        u = np.matmul(K, error);
        # u = -np.matmul(K, state);
        # u1 = np.matmul(K, goal);
        # u += u1
        log.append(np.copy(state))
        control_inputs.append(np.copy(u))
        state += (np.matmul(A, state) + np.matmul(B, u)) * dt;
        t += 1;
    fig, ax = plt.subplots(B.shape[1] + A.shape[0], sharex = True)

    ax[0].plot(np.arange(0, time_steps) * dt, log[:-1,0])
    ax[0].set(xlabel='Time(s)', ylabel='Height (m)')
    ax[1].plot(np.arange(0, time_steps) * dt, control_inputs[:-1,0])
    ax[1].set(xlabel='Time(s)', ylabel='Control Effort (N)')
    plt.show()
