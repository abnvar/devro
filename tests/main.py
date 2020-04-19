import cv2
import simpy
import numpy as np

from devro.config.envconfig import setup
from devro.env.mapGen import getRandomMap
from devro.env.components import Threader, Lidar, Bot
from devro.slam.slam import GeneticSLAM

setup(pixelSpan = 720, distSpan = 10)
envMap = getRandomMap()
lidar = Lidar(360, 6, 0.05)
env = simpy.RealtimeEnvironment(strict=True)
bot = Bot(env=env, dt=100, envMap=envMap, lidar=lidar, wheelDist=0.2, visualise=1)

simThread = Threader("Simulation Thread", env, bot)
simThread.start()

slam = GeneticSLAM(lidar)

import time
bot.setVel(0.39, 0.4)
A = bot.scan()
slam.update(A)
time.sleep(2)
B = bot.scan()
slam.update(B)
