import simpy
import numpy as np
from random import randint
from mapGen import getRandomMap
from math import sin, cos
import threading
import time
import cv2


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
        self.env.run(until=bot.driveProc)


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

    def __init__(self, ppr = 360, range_ = 2, resolution):
        self.ppr = ppr
        self.range_ = range_
        self.resolution = resolution


class Bot():
    def __init__(self, env, dt, envMap, lidar, wheelDist, visualise = 1):
        self.env = env
        self.wheelDist = int(wheelDist * (pixelSpan/distSpan))
        self.lidar = lidar
        self.dt = dt/1000   # converting to milliseconds
        self.x = 50
        self.y = pixelSpan - 50
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
        while self.x > clearance and self.y > clearance and self.x < pixelSpan-clearance and self.y < pixelSpan-clearance:
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
        self.vl = vl * (pixelSpan/distSpan)
        self.vr = vr * (pixelSpan/distSpan)

    def scan(self):
        scanList = []
        resolution = self.lidar.resolution
        alpha = 2*np.pi/self.lidar.ppr
        for k in range(self.lidar.ppr):
            i, j = self.x, self.y
            step = 0
            while self.map_[i][j] != 0 and resolution*step < self.lidar.range_:
                i += resolution*cos(alpha*k)
                j += resolution*sin(slpha*k)
                step += 1
            if self.map_[i][j] == 0:
                scanList.append(resolution*step)
            else:
                scanList.append(np.inf)

        print(scanList)




    def showSimulation(self):
        canvas = np.array(self.map_)
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), self.wheelDist//2, (0,0,0), -self.wheelDist//2)
        canvas = cv2.line(canvas, (int(self.x), int(self.y)), (int(self.x+self.wheelDist*cos(self.theta)), int(self.y+self.wheelDist*sin(self.theta))), (0,0,0), 2)
        cv2.imshow('win', canvas)
        cv2.waitKey(1)


pixelSpan, distSpan = (720, 5)    # px, meters
envMap = getRandomMap(pixelSpan, distSpan)
lidar = Lidar(360, 2, 0.05)
env = simpy.RealtimeEnvironment(strict=True)
bot = Bot(env=env, dt=100, envMap=envMap, lidar=lidar, wheelDist=0.1, visualise=1)

simThread = Threader("Simulation Thread", env, bot)
simThread.start()
bot.scan()
# bot.setVel(0.39, 0.4)
