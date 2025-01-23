import matplotlib.pyplot as plt
import control
import numpy as np
from trajectory import Trajectory
import math
import scipy.stats as stats

mass = 100
mass0 = mass
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1 / (mass)]])
Q = np.diag(np.array([10, 30]))
R = np.diag(np.array([.001]))
K, S, E = control.lqr(A, B, Q, R)
def create_traj(func, dt, domain):
        points = []
        for t in range(0, int((domain[1] - domain[0]) / dt)):
            points.append(func(t * dt + domain[0]))
        return Trajectory(points, dt)

print(K)

dt = 0.01;
total_sim_time = 35;
time_steps = (int)(total_sim_time / dt);
t = 0;
log = []
control_inputs = []
state = np.transpose(np.array([10., 0.]))

goal = np.transpose(np.array([0., 0.]))
thing = np.transpose(np.array([0., 1.]))
g = np.transpose(np.array([0, -9.81]))
def sin_path(t):
    return 5 *math.sin(t)
def step_path(t):
    if(math.floor(t) % 5 == 0):
        return 0
    return 2 
def sigmoid_path(t):
    """
    A sigmoid-like function that starts slow, speeds up, and slows down again, 
    going from 100 to 0, reaching 0 at t=15 and staying there.
    
    Parameters:
    t (float): Time input.
    
    Returns:
    float: Value of f(t).
    """
    # t = t + 5
    if t >= 15:
        return 0  # Ensures the function stays at 0 after t=15
    
    # Logistic function scaled to [100, 0] over [0, 15]
    k = 1  # Adjust steepness
    midpoint = 7.5  # Midpoint of the sigmoid
    return 100 / (1 + np.exp(k * (t - midpoint)))
def normal_path(t):
    time_to_land = 60
    if t >= time_to_land:
        return 0
    variance = time_to_land
    sigma = math.sqrt(variance)
    mu = 0
    multiplier = (10.0 / stats.norm.pdf(0, mu, sigma))
    return stats.norm.pdf(t, mu, sigma ) * multiplier
maneuver_complete = False
def hop_traj(t):
    if t >= 30:
        # global maneuver_complete
        # maneuver_complete = True
        return 1
    return 100 * math.sin( ((t-7.5) * math.pi) / 15) + 101
traj = create_traj(normal_path, dt, (0, total_sim_time + 10))
goals = []
mass_log = []
gone_up = False
u = np.array([0])
pu = np.array([0])
t_prev = 0
t_curr = 0
freq = 30
while(t <= time_steps ):
    # noise = ((np.random.rand() - .5) * 100) 
    noise = 0
    goal[0] , goal[1]= traj.sample(t * dt)
    goals.append(np.copy(goal))
    if (t % freq == 0):
        # print(goal)
        pu = np.copy(u)
        t_prev = t_curr
        t_curr = t
        error = goal - state;
        u = np.matmul(K, error);
        u[0] = u[0] + 9.81 * mass0
        # u[0] = min(max(u[0], mass0 * .7 * 9.81), mass0 * 1.2 * 9.81)
    control_input = np.array([np.interp(t - freq, [t_prev, t_curr], [pu[0], u[0]])])
    print(control_input, t, t_prev, t_curr)
    # control_inputs.append(np.copy(control_input))
    control_inputs.append(np.copy(state[1]))   
    # mass -= (u[0] / (150 * 9.81)) * dt
    mass = max(mass0 * .5, mass)
    mass_log.append(np.copy(control_input) / mass0)
    # print("mass", mass)
    log.append(np.copy(state))  
    state += (np.matmul(A, state) + thing * ((control_input[0] + mass * (-9.81) + noise) / mass)) * dt;
    if(state[0] > 15):
        gone_up = True
    # if(state[0] < .05):
    #     print("ended early")
    #     time_steps = t
    #     break
    t += 1;

log = np.array(log)
control_inputs = np.array(control_inputs)
goals = np.array(goals)
print(goals[:, 0]) 
thingle = 0
for i in mass_log:
    thingle += i * dt
print("Delta V", thingle)
fig, ax = plt.subplots(3, sharex = True)
ax[0].plot(np.arange(0, time_steps) * dt, log[:-1,0], color = 'g', label = 'current height')
ax[0].plot(np.arange(0, time_steps) * dt, goals[:-1,0], label = 'goal')
ax[0].axhline(y=0, color='black') 
ax[0].legend()
ax[0].set(xlabel='Time(s)', ylabel='Height (m)')
ax[1].plot(np.arange(0, time_steps) * dt, control_inputs[:-1])
ax[1].set(xlabel='Time(s)', ylabel='Control Effort (N)')
ax[2].plot(np.arange(0, time_steps) * dt, mass_log[:-1])
ax[2].set(xlabel='Time(s)', ylabel='Mass (kg)')
print(B.shape)
print("average mass: ", sum(mass_log) / len(mass_log))
print("final_mass: ", mass_log[len(mass_log) - 1])
print("min height: ", log[:-1,0].min());
plt.show()

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



# for a given flight path, find the delta-v corresponding to it
## total acceleration at all points, add g aceel (+), integrate it wrt time