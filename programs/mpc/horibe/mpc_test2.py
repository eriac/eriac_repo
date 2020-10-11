import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import cvxopt
from cvxopt import matrix
import scipy.linalg

import time
import math

"""
https://myenigma.hatenablog.com/entry/2017/02/07/084922
https://qiita.com/taka_horibe/items/47f86e02e2db83b0c570
"""

class System:
    def __init__(self, delay):
        self.state = np.array([[0], [0]], dtype="float") # (pos, vel)T
        self.acc = [0]
        self.dt = 0.01
        self.tau = 1.0 / delay
        self.A = np.array([[1, self.dt], [0, 1 - self.tau*self.dt]], dtype="float")
        self.B = np.array([[0], [self.tau*self.dt]], dtype="float")
        # self.last_u = 0.0

    def tick(self, input_u):
        current_s = self.state[:,-1].reshape(-1, 1)
        # real_u = min(max(input_u, self.last_u-10.0*self.dt), self.last_u+10.0*self.dt)

        next_s = np.matmul(self.A, current_s) + self.B * input_u
        # next_s = np.matmul(self.A, current_s) + self.B * real_u
        self.state = np.hstack((self.state, next_s))
        self.acc = np.hstack((self.acc, input_u))
        # self.acc = np.hstack((self.acc, real_u))
        # self.last_u = real_u

    def getSize(self):
        return self.state.shape[1]
    
    def getState(self):
        return self.state[:,-1]

class ModelingTool:
    def __init__(self, A, B, N, Q, R, xmax=None, umax=None, dumax=None):
        """
        solve MPC with modeling tool for test
        """
        (nx, nu) = B.shape

        # mpc calculation
        self.x = cvxpy.Variable((nx, N + 1))
        self.u = cvxpy.Variable((nu, N))

        costlist = 0.0
        constrlist = []

        self.r = cvxpy.Parameter((nx, N))
        self.x0 = cvxpy.Parameter((nx, 1))
        self.last_u = cvxpy.Parameter((nu, 1))

        for t in range(N):
            costlist += 0.5 * cvxpy.quad_form(self.x[:, t]-self.r[:,t], Q)
            costlist += 0.5 * cvxpy.quad_form(self.u[:, t], R)

            constrlist += [self.x[:, t + 1] == A @ self.x[:, t] + B @ self.u[:, t]]

            if xmax is not None:
                constrlist += [self.x[:, t] >= -xmax[:, 0]]
                constrlist += [self.x[:, t] <= xmax[:, 0]]

        costlist += 0.5 * cvxpy.quad_form(self.x[:, N], Q)  # terminal cost

        if xmax is not None:
            constrlist += [self.x[:, N] >= -xmax[:, 0]]
            constrlist += [self.x[:, N] <= xmax[:, 0]]

        if umax is not None:
            constrlist += [self.u >= -umax]  # input constraints
            constrlist += [self.u <= umax]  # input constraints

        # add
        if dumax is not None:
            constrlist += [self.u[:,0:1] - self.last_u >= -dumax]
            constrlist += [self.u[:,0:1] - self.last_u <= dumax]
            for i in range(N-1):
                constrlist += [self.u[:,i+1:i+2] - self.u[:,i:i+1] >= -dumax]
                constrlist += [self.u[:,i+1:i+2] - self.u[:,i:i+1] <= dumax]

        constrlist += [self.x[:, 0] == self.x0[:, 0]]  # inital state constraints

        self.prob = cvxpy.Problem(cvxpy.Minimize(costlist), constrlist)

    def solve(self, x0, r, last_u):
        self.x0.value = x0
        self.r.value = r
        self.last_u.value = last_u
        self.prob.solve()
        return self.x.value, self.u.value

class MpcControl:
    def __init__(self, delay, vmax, amax):
        self.dt = 0.1
        self.tau = 1.0 / delay
        self.A = np.array([[1.0, self.dt], [0, 1.0 - self.tau*self.dt]])
        self.B = np.array([[0], [self.tau*self.dt]])
        (nx, nu) = self.B.shape
        self.N = 10  # number of horizon
        self.Q = np.matrix([[100,0],[0,1]])
        self.R = np.eye(nu)*0.1
        self.target = target
        self.last_u = np.zeros((1,1))
        self.modeling_tool =ModelingTool(self.A, self.B, self.N, self.Q, self.R, umax=vmax, dumax=amax*self.dt)

    def control(self, target, state):
        x0 = np.array([[state[0]], [state[1]]])
        r = np.zeros((2,self.N))
        for i in range(self.N):
            if i < len(target):
                r[0,i] = target[i]
                r[1,i] = 0
            else:
                r[0,i] = target[-1]
                r[1,i] = 0
        
        x, u = self.modeling_tool.solve(x0, r, self.last_u)
        self.last_u = u[0:1,0:1]
        return u[0,0]
        
target = np.zeros((1,2))
time_size = 700
# for i in range(time_size):
#     x = 0
#     y = 0
#     if i < 100:
#         x = 0.0
#         y = 0.0
#     elif i < 200:
#         x = 0.0 + (i-100)* 0.01
#         y = 0.0
#     elif i < 400:
#         x = 1.0-(i-200)*0.01
#         y = (i-200)*0.01
#     elif i < 600:
#         x = -1.0+(i-400)*0.005
#         y = 2.0-(i-400)*0.005
#     else:
#         x = 0.0
#         y = 1.0
#     target = np.append(target, np.array([[x,y]]),axis=0) 

for i in range(time_size):
    x = 0
    y = 0
    if i < 100:
        x = 0.0
        y = 0.0
    elif i < 200:
        x = 0.0 + (i-100)* 0.01
        y = 0.0
    elif i < 300:
        x = 1.0
        y = 0.0+(i-200)*0.01
    elif i < 400:
        x = 1.0-(i-300)*0.01
        y = 1.0
    elif i < 500:
        x = 0.0
        y = 1.0-(i-400)*0.01
    else:
        x = 0.0
        y = 0.0
    # x+=1.0
    # y+=1.0
    target = np.append(target, np.array([[x,y]]),axis=0) 

# for i in range(time_size):
#     if i < 100:
#         x = -1.0
#         y = -1.0
#     elif i < 200:
#         x = -1.0 + (i-100)* 0.01
#         y = -1.0
#     elif i < 514:
#         theta = (i-200) * 0.01 -1.5707
#         x = 0.0 + math.cos(theta)
#         y = 0.0 + math.sin(theta)
#     elif i < 614:
#         x = 0.0 - (i-514)*0.01
#         y = 1.0
#     else:
#         x = -1.0
#         y = 1.0
#     # x += 2.0
#     target = np.append(target, np.array([[x,y]]),axis=0) 

# MPC control
system_x = System(0.2)
system_y = System(0.2)
mpc_x = MpcControl(0.2, 2.5, 2.0)
mpc_y = MpcControl(0.2, 2.5, 2.0)

start = time.time()
for i in range(time_size):
    if i%10 == 0:
        state_x = system_x.getState()
        state_y = system_y.getState()
        r_x = []
        r_y = []
        for j in range(10):
            index = i + j *10
            if  index < len(target):
                r_x.append(target[index,0])
                r_y.append(target[index,1])
            else:
                r_x.append(target[-1,0])
                r_y.append(target[-1,1])

        output_x = mpc_x.control(r_x, state_x)
        output_y = mpc_y.control(r_y, state_y)
    system_x.tick(output_x)
    system_y.tick(output_y)
end = time.time()
print ("elapsed_time:{0}".format(end - start) + "[sec]")


sys_size = system_x.getSize()
t = np.linspace(0, sys_size * system_x.dt, sys_size)
fig = plt.figure()

ax_1 = fig.add_subplot(311)
ax_1.plot(t, system_x.acc, label="acc")
ax_1.plot(t, system_x.state[1,:], label="vel")
ax_1.plot(t, system_x.state[0,:], label="pos")
ax_1.plot(t, target[:,0], label="target")

ax_2 = fig.add_subplot(312)
ax_2.plot(t, system_y.acc, label="acc")
ax_2.plot(t, system_y.state[1,:], label="vel")
ax_2.plot(t, system_y.state[0,:], label="pos")
ax_2.plot(t, target[:,1], label="target")

ax_3 = fig.add_subplot(313)
ax_3.plot(system_x.state[0,:], system_y.state[0,:], label="pos")
ax_3.plot(target[:,0], target[:,1], label="target")

plt.legend(loc='best')
plt.show()
