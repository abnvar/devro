import cv2
import simpy
import numpy as np

import devro
from devro.utilities import Scheduler
from devro.algorithms.pathPlanning import Astar
from devro.env.mapGen import getRandomMap
from devro.env.components import Simulation, Bot
from devro.env.sensors import Scanner, Encoder
from devro.env.actuators import Motor

envMap = getRandomMap(pixelSpan = 720, distSpan = 10, randbase = 1)

lMotor = Motor(mode='rpm', wheelRadius = 0.1)
rMotor = Motor(mode='rpm', wheelRadius = 0.1)
scanner = Scanner(ppr = 360, range_ = 3, resolution = 0.05, fieldAngle = 2*np.pi)
bot = Bot(leftMotor=lMotor, rightMotor=rMotor, scanner=scanner, wheelDist=0.2)

sim = Simulation(pixelSpan = 720, distSpan = 10, dt = 42, envMap = envMap, bot = bot, visualize = True)
sim.addDynamicObstacles(qty=0, radiusRange=(10,20), maxVelocity=10)
sim.begin()

bot.setVel(38.5, 40)
s = Scheduler()
s.setInterval(func=scanner.scan, dt=0.1)

def updateAstar():
    img = np.asarray(envMap, np.uint8)
    p = Astar(startCoord=(bot.x, bot.y), destCoord=(bot.destX, bot.destY), map_=img, optimize = False)
    trajectory = p.find()
    sim.win.setSlamFrame(img)

s.setTimer(func=updateAstar, dt=3)

sim.hold()

# sim.active = False  # To end the simulation via script
# sim.reset()   # reset env
