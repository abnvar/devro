import simpy
import numpy as np
import threading
import time
import cv2
from math import sin, cos

from devro.config.envconfig import envdata

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
    def __init__(self, env, dt, envMap, lidar, wheelDist, visualise=1):
        self.env = env
        self.wheelDist = int(wheelDist * (envdata.pixelSpan/envdata.distSpan))
        self.lidar = lidar
        self.dt = dt/1000   # converting to milliseconds
        self.x = 50
        self.y = envdata.pixelSpan - 50
        self.destX = envdata.pixelSpan - 50
        self.destY = 50
        self.vl = 0
        self.vr = 0
        self.theta = 0
        self.omega = 0
        self.vx = 0
        self.vy = 0
        self.map_ = envMap
        self.visualise = visualise
        self.driveProc = env.process(self.drive(env))

    def drive(self, env):
        clearance = self.wheelDist/2
        while self.x > clearance and self.y > clearance and self.x < envdata.pixelSpan-clearance and self.y < envdata.pixelSpan-clearance:
            v = (self.vl + self.vr)/2
            self.vx = v*cos(self.theta)
            self.vy = v*sin(self.theta)
            self.x += self.vx*self.dt
            self.y += self.vy*self.dt
            self.omega = (self.vl-self.vr)/(self.wheelDist)
            self.theta += self.omega*self.dt
            if self.visualise == 1:
                self.showSimulation()
            yield env.timeout(self.dt)

    def setVel(self, vl, vr):
        self.vl = vl * (envdata.pixelSpan/envdata.distSpan)
        self.vr = vr * (envdata.pixelSpan/envdata.distSpan)

    def scan(self):
        a = time.perf_counter()
        scanList = []
        resolution = self.lidar.resolution
        alpha = 2*np.pi/self.lidar.ppr
        for k in range(self.lidar.ppr):
            j, i = self.x, self.y
            step = 0
            while (i > 0 and i < envdata.pixelSpan and j > 0 and j < envdata.pixelSpan) and resolution*step < self.lidar.range_ and self.map_[int(i)][int(j)] != 0:
                i += resolution*cos(np.pi-self.theta+alpha*k)
                j += resolution*sin(np.pi-self.theta+alpha*k)
                step += 1
            if (i > 0 and i < envdata.pixelSpan and j > 0 and j < envdata.pixelSpan) and self.map_[int(i)][int(j)] == 0:
                scanList.append(resolution*step *(envdata.distSpan/envdata.pixelSpan))
            else:
                scanList.append(np.inf)

        return scanList

    def showSimulation(self):
        canvas = np.stack((self.map_,)*3, axis=-1)
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), self.wheelDist//2, (0,0,0), -self.wheelDist//2)    # Robot
        canvas = cv2.putText(canvas, 'x', (self.destX, self.destY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), int(self.lidar.range_), (255,0,0), 2)      # Lidar circle
        canvas = cv2.line(canvas, (int(self.x), int(self.y)), (int(self.x+self.wheelDist*cos(self.theta)), int(self.y+self.wheelDist*sin(self.theta))), (0,0,0), 2)    # direction line
        scanList = self.scan()
        scanImg = self.imagifyScan(scanList)
        cv2.imshow('env', canvas)
        cv2.waitKey(1)
        cv2.imshow('lidar', scanImg)
        cv2.waitKey(1)

    def imagifyScan(self, scanList):
        blank = np.zeros((envdata.pixelSpan,envdata.pixelSpan))
        scaler = envdata.pixelSpan/envdata.distSpan
        blank = cv2.circle(blank, (envdata.pixelSpan//2, envdata.pixelSpan//2), 4, (255,255,255), -4)
        for i in range(self.lidar.ppr):
            if scanList[i] != np.inf:
                center = (envdata.pixelSpan//2+int(scaler*scanList[i]*sin((i-90)*np.pi/180)), envdata.pixelSpan//2+int(scaler*scanList[i]*cos((i-90)*np.pi/180)))
                blank = cv2.circle(blank, center, 2, (255,255,255), -2)

        return blank
