import cv2
import simpy
import numpy as np

import devro
from devro.utilities import Scheduler
from devro.env.mapGen import getRandomMap
from devro.env.components import Simulation, Bot
from devro.env.sensors import Scanner, Encoder
from devro.env.actuators import Motor

envMap = getRandomMap(pixelSpan = 720, distSpan = 10, randbase = 1)

lEncoder, rEncoder = Encoder(), Encoder()
lMotor = Motor(mode='rpm', wheelRadius = 0.1, encoder = lEncoder)
rMotor = Motor(mode='rpm', wheelRadius = 0.1, encoder = rEncoder)
scanner = Scanner(ppr = 360, range_ = 3, resolution = 0.05, minDist = 0.5)
bot = Bot(leftMotor=lMotor, rightMotor=rMotor, scanner=scanner, wheelDist=0.2)

sim = Simulation(pixelSpan = 720, distSpan = 10, dt = 42, envMap = envMap, bot = bot, visualize = True , dynamicObstacles=3)
sim.addDynamicObstacles(qty=3, radiusRange=(10,20), maxVelocity=10)
sim.begin()

bot.setVel(38.5, 40)
s = Scheduler()
s.setInterval(func=scanner.scan, dt=0.1)

sim.hold()

# sim.active = False  # To end the simulation via script
# sim.reset()   # reset env
