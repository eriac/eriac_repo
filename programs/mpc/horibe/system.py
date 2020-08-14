import numpy as np
import matplotlib.pyplot as plt

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

class Control:
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


system = System()

target = [0]

time_size = 700
for i in range(time_size):
    if i < 50:
        target.append(0.0)
    elif i < 200:
        target.append(1.0)
    elif i < 400:
        target.append(1.0+(i-200)*0.005)
    else:
        target.append(1.0)
pid_control = Control(target)

# control
for i in range(time_size):
    if i%10 == 0:
        state = system.getState()
        output = pid_control.control(i, state)

    system.tick(output)

error = [0]
for i in range(time_size):
    error.append(system.pos[i] - target[i])

sys_size = system.getSize()
x = np.linspace(0, sys_size * system.dt, sys_size)
fig = plt.figure()

ax_1 = fig.add_subplot(211)
ax_1.plot(x, system.acc, marker='.', markersize=10, label="acc")
ax_1.plot(x, system.vel, marker='.', markersize=10, label="vel")
ax_1.plot(x, system.pos, marker='.', markersize=10, label="pos")
ax_1.plot(x, target, marker='.', markersize=10, label="target")
plt.legend(loc='best')

ax_2 = fig.add_subplot(212)
ax_2.plot(x, error, marker='.', markersize=10, label="error")
plt.legend(loc='best')

plt.show()
