import simpy
import numpy as np
import threading
import cv2
from math import sin, cos

from devro.config.envconfig import envdata
from devro.config.envconfig import setup
from devro.visualization import display

class Threader (threading.Thread):
    '''
    Utility class for environment threading

    ...

    Attributes
    ----------
    threadName : str
        the name of the thread
    env : simpy environment object
        the simpy environment object to be threaded
    bot : class Bot object
        the bot object to be run in the environment

    Methods
    -------
    run()
        runs the environment in the named thread
    '''

    def __init__(self, threadName, env, bot):
        threading.Thread.__init__(self)
        self.threadName = threadName
        self.env = env
        self.bot = bot

    def run(self):
        self.env.run(until=self.bot.driveProc)


class Lidar():
    '''
    Class to hold lidar properties

    ...

    Attributes
    ----------
    ppr : int
        points per revolution (default 360)
    range : int
        maximum detection range of the lidar (in meters) (default 2)
    resolution : float
        distance measuring accuracy/resolution (in meters) (default 0.05)

    Methods
    -------
    None
    '''

    def __init__(self, ppr=360, range_=2, resolution=0.05):
        self.ppr = ppr
        self.range_ = range_ * (envdata.pixelSpan/envdata.distSpan)
        self.resolution = resolution * (envdata.pixelSpan/envdata.distSpan)


class Bot():
    def __init__(self, lidar, wheelDist):
        self.lidar = lidar
        self.wheelDist = wheelDist
        self.vl = 0
        self.vr = 0
        self.theta = 0
        self.omega = 0
        self.vx = 0
        self.vy = 0

        self.x = None
        self.y = None
        self.destX = None
        self.destY = None
        self.sim = None
        self.env = None
        self.map_ = None
        self.dt = None

    def setInitPos(self, x, y):
        self.x = x
        self.y = y

    def setDestPos(self, x, y):
        self.destX = x
        self.destY = y

    def attachSim(self, sim, env, envMap, dt):
        self.sim = sim
        self.env = env
        self.map_ = envMap
        self.dt = dt

        self.setInitPos(50, sim.pixelSpan - 50)
        self.setDestPos(sim.pixelSpan - 50, 50)
        self.wheelDist = int(self.wheelDist * (sim.pixelSpan/sim.distSpan))

        self.driveProc = self.env.process(self.drive(env))

    def drive(self, env):
        clearance = self.wheelDist/2
        while self.x > clearance and self.y > clearance and self.x < self.sim.pixelSpan-clearance and self.y < self.sim.pixelSpan-clearance:
            v = (self.vl + self.vr)/2
            self.vx = v*cos(self.theta)
            self.vy = v*sin(self.theta)
            self.x += self.vx*self.dt
            self.y += self.vy*self.dt
            self.omega = (self.vl-self.vr)/(self.wheelDist)
            self.theta += self.omega*self.dt

            if self.sim.visualize == True:
                self.sim.showSimulation()

            yield env.timeout(self.dt)

    def setVel(self, vl, vr):
        self.vl = vl * (self.sim.pixelSpan/self.sim.distSpan)
        self.vr = vr * (self.sim.pixelSpan/self.sim.distSpan)

    def scan(self, visualize = True):
        scanList = []
        resolution = self.lidar.resolution
        alpha = 2*np.pi/self.lidar.ppr

        for k in range(self.lidar.ppr):
            j, i = self.x, self.y
            step = 0
            while (i > 0 and i < self.sim.pixelSpan and j > 0 and j < self.sim.pixelSpan) and resolution*step < self.lidar.range_ and self.map_[int(i)][int(j)] != 0:
                i += resolution*cos(np.pi-self.theta+alpha*k)
                j += resolution*sin(np.pi-self.theta+alpha*k)
                step += 1
            if (i > 0 and i < self.sim.pixelSpan and j > 0 and j < self.sim.pixelSpan) and self.map_[int(i)][int(j)] == 0:
                scanList.append(resolution*step *(self.sim.distSpan/self.sim.pixelSpan))
            else:
                scanList.append(np.inf)

        if visualize == True:
            scanImg = self.imagifyScan(scanList)
            self.sim.showLidar(scanImg)

        return scanList

    def imagifyScan(self, scanList):
        blank = np.ones((self.sim.pixelSpan,self.sim.pixelSpan))*255
        scaler = self.sim.pixelSpan/self.sim.distSpan
        blank = cv2.circle(blank, (self.sim.pixelSpan//2, self.sim.pixelSpan//2), 4, (0,0,0), -4)
        for i in range(self.lidar.ppr):
            if scanList[i] != np.inf:
                center = (self.sim.pixelSpan//2+int(scaler*scanList[i]*sin((i-90)*np.pi/180)), self.sim.pixelSpan//2+int(scaler*scanList[i]*cos((i-90)*np.pi/180)))
                blank = cv2.circle(blank, center, 2, (0,0,0), -2)

        return blank

    def plotBot(self, canvas):
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), self.wheelDist//2, (0,0,0), -self.wheelDist//2)    # Robot
        canvas = cv2.putText(canvas, 'x', (self.destX, self.destY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), int(self.lidar.range_), (255,0,0), 2)      # Lidar circle
        canvas = cv2.line(canvas, (int(self.x), int(self.y)), (int(self.x+self.wheelDist*cos(self.theta)), int(self.y+self.wheelDist*sin(self.theta))), (0,0,0), 2)    # direction line

        return canvas


class Simulation():
    def __init__(self, pixelSpan = 720, distSpan = 10, dt = 100, envMap = None, bot = None, visualize = True):
        self.pixelSpan = pixelSpan
        self.distSpan = distSpan
        self.dt = dt/1000   # converting to milliseconds
        self.envMap = envMap
        self.bot = bot
        self.visualize = visualize
        self.env = simpy.RealtimeEnvironment(strict=True)
        self.win = display.Window('Simulation', height = self.pixelSpan, dt = dt)

        bot.attachSim(self, self.env, self.envMap, self.dt)


    def begin(self):
        simThread = Threader("Simulation Thread", self.env, self.bot)
        simThread.start()

    def showSimulation(self):
        canvas = np.stack((self.envMap,)*3, axis=-1)
        canvas = self.bot.plotBot(canvas)
        self.win.setEnvFrame(canvas)

    def showLidar(self, img):
        self.win.setLidarFrame(img)
