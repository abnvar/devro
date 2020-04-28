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
scanner = Scanner(ppr = 360, range_ = 6, resolution = 0.05)
bot = Bot(leftMotor=lMotor, rightMotor=rMotor, scanner=scanner, wheelDist=0.2)
sim = Simulation(pixelSpan = 720, distSpan = 10, dt = 42, envMap = envMap, bot = bot, visualize = True)
sim.begin()

import time
bot.setVel(77, 80)

s = Scheduler()
s.setInterval(func=bot.scan, dt=0.1)

sim.hold()

# sim.active = False  # To end the simulation via script
# sim.reset()   # reset env
