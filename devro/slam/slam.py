import config
import numpy as np
from math import sin, cos
import random

class geneticSLAM():
    def __init__(self, lidar):
        self.lidar = lidar
        self.map = None

    def scanToCoords(self, scanList):
        coords = []
        scaler = config.pixelSpan/config.distSpan
        for i in range(self.lidar.ppr):
            if scanList[i] != np.inf:
                pt = [config.pixelSpan//2+int(scaler*scanList[i]*sin((i-90)*np.pi/180)), config.pixelSpan//2+int(scaler*scanList[i]*cos((i-90)*np.pi/180))]
                coords.append(pt)
        return np.array(coords)

    def match(self, coords):
        epsilon = 1 * (config.pixelSpan/config.distSpan)


    def update(self, scan):
        coords = self.scanToCoords(scan)
        print(coords)

        if self.map == None:
            self.map = coords
        else:
            self.match(coords)
