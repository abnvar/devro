import simpy
import numpy as np
import matplotlib.pyplot as plt

import config
from mapGen import getRandomMap
from components import Threader, Lidar, Bot


config.setup()
envMap = getRandomMap(config.pixelSpan, config.distSpan)
lidar = Lidar(360, 6, 0.05)
env = simpy.RealtimeEnvironment(strict=True)
bot = Bot(env=env, dt=100, envMap=envMap, lidar=lidar, wheelDist=0.2, visualise=1)

simThread = Threader("Simulation Thread", env, bot)
simThread.start()

import time
bot.setVel(0.39, 0.4)
