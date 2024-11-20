import matplotlib.pyplot as plt
import control
import numpy as np

A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
Q = np.diag(np.array([2, 1]))
R = np.diag(np.array([1]))

K, S, E = control.lqr(A, B, Q, R)
print(K)

dt = 0.01;
total_sim_time = 10;
time_steps = (int)(total_sim_time / dt);
t = 0;
log = []
control_inputs = []
state = np.transpose(np.array([0., 0.]))

goal = np.transpose(np.array([10., 0.]))
while(t <= time_steps):
    error = goal - state;
    u = np.matmul(K, error);
    log.append(np.copy(state))
    control_inputs.append(np.copy(u))
    state += (np.matmul(A, state) + np.matmul(B, u)) * dt;
    t += 1;
log = np.array(log)
control_inputs = np.array(control_inputs)
print(log[:, 0]) 


fig, ax = plt.subplots(2, sharex = True)

ax[0].plot(np.arange(0, time_steps) * dt, log[:-1,0])
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
        error = goal - state;
        u = np.matmul(K, error);
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
