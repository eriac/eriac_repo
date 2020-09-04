from scipy import integrate
import matplotlib.pyplot as plt
import numpy as np

class Coo():
    def __init__(self,_x,_y):
        self.x = _x
        self.y = _y

class Bezier():
    def __init__(self,_p0,_p1,_p2,_p3):
        self.p = [0]*4
        self.p[0] = _p0
        self.p[1] = _p1
        self.p[2] = _p2
        self.p[3] = _p3

        self.fig = plt.figure(figsize=(10,5))
        self.ax = self.fig.add_subplot(121)#111 1x1 
        self.ax.set_xlim(-1.0 , 11.0)
        self.ax.set_ylim(-1.0 , 11.0)

        self.dax = self.fig.add_subplot(122)#111 1x1 

        self.t = np.arange(0,1,0.01)

        self.x = (self.t**3)*self.p[3].x + 3*(self.t**2)*(1-self.t)*self.p[2].x + 3*self.t*((1-self.t)**2)*self.p[1].x + ((1-self.t)**3)*self.p[0].x
        self.y = (self.t**3)*self.p[3].y + 3*(self.t**2)*(1-self.t)*self.p[2].y + 3*self.t*((1-self.t)**2)*self.p[1].y + ((1-self.t)**3)*self.p[0].y

        self.dx = 3*((self.t**2)*(self.p[3].x-self.p[2].x)+2*self.t*(1-self.t)*(self.p[2].x-self.p[1].x)+((1-self.t)**2)*(self.p[1].x-self.p[0].x))
        self.dy = 3*((self.t**2)*(self.p[3].y-self.p[2].y)+2*self.t*(1-self.t)*(self.p[2].y-self.p[1].y)+((1-self.t)**2)*(self.p[1].y-self.p[0].y))

        self.ddx = 6*(self.t*(self.p[3].x-2*self.p[2].x+self.p[1].x)+(1-self.t)*(self.p[2].x-2*self.p[1].x+self.p[0].x))
        self.ddy = 6*(self.t*(self.p[3].y-2*self.p[2].y+self.p[1].y)+(1-self.t)*(self.p[2].y-2*self.p[1].y+self.p[0].y))

        self.artists = [0]*4
        self.artists[0], = self.ax.plot([self.p[0].x],[self.p[0].y],marker='.',markersize=10,label='p[0]')
        self.artists[1], = self.ax.plot([self.p[1].x],[self.p[1].y],marker='.',markersize=10,label='p[1]')
        self.artists[2], = self.ax.plot([self.p[2].x],[self.p[2].y],marker='.',markersize=10,label='p[2]')
        self.artists[3], = self.ax.plot([self.p[3].x],[self.p[3].y],marker='.',markersize=10,label='p[3]')
        self.ax_artist, = self.ax.plot(self.x,self.y)

        dp = [((x_a-x_b)**2 + (y_a-y_b)**2)*100 for x_a,x_b,y_a,y_b in zip(self.x[:-1],self.x[1:],self.y[:-1],self.y[1:])]
        adp = [(i**2 + j**2)**0.5 for i,j in zip(self.dx,self.dy)]
        addp = [(i**2 + j**2)**0.5 for i,j in zip(self.ddx,self.ddy)]

        R = (self.dx*self.ddy - self.ddx*self.dy) / ((self.dx**2 + self.dy**2)**1.5)

        self.dax.set_xlim(-0.1 , 1.5)
        # self.dax.set_ylim(-1.0 , 40)
        dp.append(0.0)

        integrate_simps = integrate.simps(self.y, self.x)
        self.dax_artist, = self.dax.plot(self.t,R)

        self.dax.plot([0.33,0.33],[0,1])

        dp_max_index = [i for i, x in enumerate(dp) if x == max(dp)]
        adp_max_index = [i for i, x in enumerate(adp) if x == max(adp)]

        # circle
        radius = 5.0
        th = np.arange(0,0.5,0.01) * np.pi
        cx = 10.0 - radius + radius * np.sin(th) 
        cy = radius - radius * np.cos(th)
        self.ax.plot(cx, cy)


    def change(self):
        self.x = (self.t**3)*self.p[3].x + 3*(self.t**2)*(1-self.t)*self.p[2].x + 3*self.t*((1-self.t)**2)*self.p[1].x + ((1-self.t)**3)*self.p[0].x
        self.y = (self.t**3)*self.p[3].y + 3*(self.t**2)*(1-self.t)*self.p[2].y + 3*self.t*((1-self.t)**2)*self.p[1].y + ((1-self.t)**3)*self.p[0].y

        self.dx = 3*((self.t**2)*(self.p[3].x-self.p[2].x)+2*self.t*(1-self.t)*(self.p[2].x-self.p[1].x)+((1-self.t)**2)*(self.p[1].x-self.p[0].x))
        self.dy = 3*((self.t**2)*(self.p[3].y-self.p[2].y)+2*self.t*(1-self.t)*(self.p[2].y-self.p[1].y)+((1-self.t)**2)*(self.p[1].y-self.p[0].y))

        self.ddx = 6*(self.t*(self.p[3].x-2*self.p[2].x+self.p[1].x)+(1-self.t)*(self.p[2].x-2*self.p[1].x+self.p[0].x))
        self.ddy = 6*(self.t*(self.p[3].y-2*self.p[2].y+self.p[1].y)+(1-self.t)*(self.p[2].y-2*self.p[1].y+self.p[0].y))

        dp = [((x_a-x_b)**2 + (y_a-y_b)**2)*100 for x_a,x_b,y_a,y_b in zip(self.x[:-1],self.x[1:],self.y[:-1],self.y[1:])]
        adp = [(i**2 + j**2)**0.5 for i,j in zip(self.dx,self.dy)]
        addp = [(i**2 + j**2)**0.5 for i,j in zip(self.ddx,self.ddy)]
        R = ((self.dx**2 + self.dy**2)**1.5)/((self.dx*self.ddy - self.ddx*self.dy)**2)**0.5

        self.ax_artist.set_data(self.x,self.y)
        self.dax_artist.set_data(self.t,R)

    def sets(self,_gco,_event):
        for i,j in zip(self.artists,self.p):
            if i == _gco:
                i.set_data(_event.xdata,_event.ydata)
                j.x = _event.xdata
                j.y = _event.ydata
                break
        self.change()


gco = None

if __name__ == '__main__':
    # a = Coo(1,1)
    # b = Coo(10,1)
    # c = Coo(1,5)
    # d = Coo(10,5)
    a = Coo(0, 0)
    b = Coo(10.0, 0.0)
    c = Coo(10.0, 0.0)
    d = Coo(10.0, 10.0)

    B = Bezier(a,b,c,d)

    def motion(event):
        global gco
        if gco is None:
            return
        B.sets(gco,event)
        plt.draw()

    def onpick(event):
        global gco
        gco = event.artist


    def release(event):
        global gco
        gco = None

    plt.connect('motion_notify_event', motion)
    plt.connect('pick_event', onpick)
    plt.connect('button_release_event', release)

    plt.show()