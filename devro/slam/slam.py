'''
    Incomplete/trash code

'''

from devro.config.envconfig import envdata
import numpy as np
from math import sin, cos
import random

class GeneticSLAM():
    def __init__(self, lidar):
        self.lidar = lidar
        self.map_ = None

    def scanToCoords(self, scanList):
        coords = []
        scaler = envdata.pixelSpan/envdata.distSpan
        for i in range(self.lidar.ppr):
            if scanList[i] != np.inf:
                pt = [envdata.pixelSpan//2+int(scaler*scanList[i]*sin((i-90)*np.pi/180)), envdata.pixelSpan//2+int(scaler*scanList[i]*cos((i-90)*np.pi/180))]
                coords.append(pt)
        return np.array(coords)

    def match(self, coords):
        epsilon = 1 * (envdata.pixelSpan/envdata.distSpan)

    def update(self, scan):
        coords = self.scanToCoords(scan)
        print(coords)

        if self.map_ == None:
            self.map_ = coords
        else:
            self.match(coords)
