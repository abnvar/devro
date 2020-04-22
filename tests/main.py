import os, sys

from os.path import dirname, join, abspath
sys.path.insert(0, abspath(join(dirname(__file__), '..')))

import cv2
import simpy
import numpy as np

from devro.env.mapGen import getRandomMap
from devro.env.components import Simulation, Lidar, Bot
from devro.slam.slam import GeneticSLAM

envMap = getRandomMap(randbase = 1)

lidar = Lidar(ppr=360, range_=2, minDist = 0.1, resolution=0.05, 
                 angleField = 4.01,
                 distScannerToRobotCenter = 0.03)

bot = Bot(lidar=lidar, wheelDist=0.2, wheelRadius=0.2)
sim = Simulation(pixelSpan = 720, distSpan = 10, dt = 100, envMap = envMap, bot = bot, visualize = True)
sim.begin()

bot.setVel(0.39, 0.4)
while True:
    A = bot.scan(visualize = True)
