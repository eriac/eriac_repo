import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import cvxopt
from cvxopt import matrix
import scipy.linalg

import time

"""
https://myenigma.hatenablog.com/entry/2017/02/07/084922
https://qiita.com/taka_horibe/items/47f86e02e2db83b0c570
"""

class System:
    def __init__(self):
        self.state = np.array([[0], [0]], dtype="float") # (pos, vel)T
        self.acc = [0]
        self.dt = 0.01
        self.tau = 1.0 / 0.5
        self.A = np.array([[1, self.dt], [0, 1 - self.tau*self.dt]], dtype="float")
        self.B = np.array([[0], [self.tau*self.dt]], dtype="float")

    def tick(self, input_u):
        current_s = self.state[:,-1].reshape(-1, 1)
        next_s = np.matmul(self.A, current_s) + self.B * input_u
        self.state = np.hstack((self.state, next_s))
        self.acc = np.hstack((self.acc, input_u))

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
            constrlist += [self.last_u - self.u[:,0:1] >= -dumax]
            constrlist += [self.last_u - self.u[:,0:1] <= dumax]
            for i in range(N-1):
                constrlist += [self.u[:,i:i+1] - self.u[:,i+1:i+2] >= -dumax]
                constrlist += [self.u[:,i:i+1] - self.u[:,i+1:i+2] <= dumax]

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
        self.Q = np.matrix([[1000,0],[0,0.01]])
        self.R = np.eye(nu)
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
        
target = [0]
time_size = 700
delta = 0.03
for i in range(time_size):
    if i < 100:
        target.append(0.0 + delta * np.random.randn()) 
    elif i < 200:
        target.append(0.0 + (i-100)* 0.01 + delta * np.random.randn())
    elif i < 400:
        target.append(1.0-(i-200)*0.01 + delta * np.random.randn())
    elif i < 600:
        target.append(-1.0+(i-400)*0.005 + delta * np.random.randn())
    else:
        target.append(0.0 + delta * np.random.randn())

# MPC control
system2 = System()
mpc_control = MpcControl(0.5, 1.5, 3.0)

start = time.time()
output=0
for i in range(time_size):
    if i%10 == 0:
        state = system2.getState()
        r = []
        for j in range(10):
            index = i + j *10
            if  index < len(target):
                r.append(target[index])
            else:
                r.append(target[-1])
        output = mpc_control.control(r, state)
    system2.tick(output)
end = time.time()
print ("elapsed_time:{0}".format(end - start) + "[sec]")


sys_size = system2.getSize()
x = np.linspace(0, sys_size * system2.dt, sys_size)
fig = plt.figure()

ax_2 = fig.add_subplot(212)
ax_2.plot(x, system2.acc, label="acc")
ax_2.plot(x, system2.state[1,:], label="vel")
ax_2.plot(x, system2.state[0,:], label="pos")
ax_2.plot(x, target, label="target")
plt.legend(loc='best')

plt.show()
