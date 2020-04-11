import simpy
import numpy as np
from random import randint
from mapGen import getRandomMap
import threading
import time
import cv2

class MyThread (threading.Thread):
   def __init__(self, threadName, env, bot):
      threading.Thread.__init__(self)
      self.threadName = threadName
      self.env = env
      self.bot = bot

   def run(self):
       self.env.run(until=bot.driveProc)


class Bot():
    def __init__(self, env, wheelDist, dt, envMap):
        self.env = env
        self.driveProc = env.process(self.drive(env))
        self.dt = dt/1000   # converting to milliseconds
        self.x = 500
        self.y = 500
        self.vx = 0
        self.vy = 0
        self.map = envMap

    def drive(self, env):
        while self.x < 720 and self.y < 720:
            print(self.x, self.y)
            self.x += self.vx*self.dt
            self.y += self.vy*self.dt
            self.showSimulation()
            yield env.timeout(self.dt)

    def setVel(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def scan(self):
        cv2.imshow('win', self.map)
        cv2.waitKey(0)

    def showSimulation(self):
        canvas = np.array(self.map)
        cv2.imshow('win', cv2.circle(canvas, (int(self.x), int(self.y)), 5, (0,0,0), 5))
        cv2.waitKey(1)


envMap = getRandomMap()
env = simpy.RealtimeEnvironment(strict=True)
bot = Bot(env, 1, 100, envMap)

simThread = MyThread("Simulation Thread", env, bot)
simThread.start()
bot.setVel(10, 10)
