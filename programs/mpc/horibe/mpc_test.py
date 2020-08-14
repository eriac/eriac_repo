import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import cvxopt
from cvxopt import matrix
import scipy.linalg

"""
https://myenigma.hatenablog.com/entry/2017/02/07/084922
https://qiita.com/taka_horibe/items/47f86e02e2db83b0c570
"""

def use_modeling_tool(A, B, N, Q, R, x0, r, umax=None, umin=None, xmin=None, xmax=None):
    """
    solve MPC with modeling tool for test
    """
    (nx, nu) = B.shape

    # mpc calculation
    x = cvxpy.Variable((nx, N + 1))
    u = cvxpy.Variable((nu, N))

    costlist = 0.0
    constrlist = []

    for t in range(N):
        costlist += 0.5 * cvxpy.quad_form(x[:, t]-r[:,t], Q)
        costlist += 0.5 * cvxpy.quad_form(u[:, t], R)

        constrlist += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        if xmin is not None:
            constrlist += [x[:, t] >= xmin[:, 0]]
        if xmax is not None:
            constrlist += [x[:, t] <= xmax[:, 0]]

    costlist += 0.5 * cvxpy.quad_form(x[:, N], Q)  # terminal cost

    if xmin is not None:
        constrlist += [x[:, N] >= xmin[:, 0]]
    if xmax is not None:
        constrlist += [x[:, N] <= xmax[:, 0]]

    if umax is not None:
        constrlist += [u <= umax]  # input constraints
    if umin is not None:
        constrlist += [u >= umin]  # input constraints

    constrlist += [x[:, 0] == x0[:, 0]]  # inital state constraints

    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constrlist)

    # prob.solve(verbose=True)
    prob.solve()

    return x.value, u.value


class System:
    def __init__(self):
        self.vel = [0]
        self.acc = [0]
        self.pos = [0]
        self.dt = 0.01

    def tick(self, input):
        self.acc.append(input)
        self.vel.append(self.vel[-1]+self.acc[-1]*self.dt)
        self.pos.append(self.pos[-1]+self.vel[-1]*self.dt)

    def getSize(self):
        return len(self.acc)
    
    def getState(self):
        return (self.pos[-1], self.vel[-1])

class PidControl:
    def __init__(self, target):
        self.last_diff = 0.0
        self.iterm = 0.0
        self.dt = 0.1
        self.target = target

    def control(self, index, state):
        pos = state[0]
        diff = pos - target[index]
        self.iterm += diff * self.dt
        dterm = (diff - self.last_diff) / self.dt
        self.last_diff = diff
        output = -4.0 * diff - 2.0 * dterm
        return min(max(output, -4.0), 4.0)

class MpcControl:
    def __init__(self, target):
        self.dt = 0.1
        self.A = np.array([[1.0, self.dt], [0, 1.0]])
        self.B = np.array([[0], [self.dt]])
        (nx, nu) = self.B.shape
        self.N = 10  # number of horizon
        self.Q = np.matrix([[10000,0],[0,0.01]])
        self.R = np.eye(nu)
        self.target = target

    def control(self, index, state):
        x0 = np.array([[state[0]], [state[1]]])
        r = np.zeros((2,self.N))
        for j in range(self.N):
            ref_index = index + j * 10
            if(ref_index < len(self.target)):
                r[0,j] = self.target[ref_index]
                r[1,j] = 0
            else:
                r[0,j] = self.target[-1]
                r[1,j] = 0
        x, u = use_modeling_tool(self.A, self.B, self.N, self.Q, self.R, x0, r, 4.0, -4.0)
        return np.array(u[0, 0])

target = [0]

time_size = 700
delta = 0.00
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
pid_control = PidControl(target)


# PID control
system1 = System()
output=0
for i in range(time_size):
    if i%10 == 0:
        state = system1.getState()
        output = pid_control.control(i, state)
    system1.tick(output)

# MPC control
system2 = System()
mpc_control = MpcControl(target)
output=0
for i in range(time_size):
    if i%10 == 0:
        state = system2.getState()
        output = mpc_control.control(i, state)
    system2.tick(output)


# error = [0]
# for i in range(time_size):
#     error.append(system.pos[i] - target[i])

sys_size = system1.getSize()
x = np.linspace(0, sys_size * system1.dt, sys_size)
fig = plt.figure()

ax_1 = fig.add_subplot(211)
ax_1.plot(x, system1.acc, label="acc")
ax_1.plot(x, system1.vel, label="vel")
ax_1.plot(x, system1.pos, label="pos")
ax_1.plot(x, target, label="target")
plt.legend(loc='best')

ax_2 = fig.add_subplot(212)
ax_2.plot(x, system2.acc, label="acc")
ax_2.plot(x, system2.vel, label="vel")
ax_2.plot(x, system2.pos, label="pos")
ax_2.plot(x, target, label="target")
plt.legend(loc='best')

plt.show()
