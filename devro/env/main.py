import cv2
import simpy
import numpy as np
import matplotlib.pyplot as plt

import config
from mapGen import getRandomMap
from components import Threader, Lidar, Bot
from icp import icp
from slam import geneticSLAM


config.setup()
envMap = getRandomMap(config.pixelSpan, config.distSpan)
lidar = Lidar(360, 6, 0.05)
env = simpy.RealtimeEnvironment(strict=False)
bot = Bot(env=env, dt=100, envMap=envMap, lidar=lidar, wheelDist=0.2, visualise=1)

simThread = Threader("Simulation Thread", env, bot)
simThread.start()

slam = geneticSLAM(lidar)

import time
bot.setVel(0.39, 0.4)
A = bot.scan()
slam.update(A)
time.sleep(2)
B = bot.scan()
slam.update(B)
