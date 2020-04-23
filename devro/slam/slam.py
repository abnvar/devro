'''
    Incomplete code
'''

import cv2
import random
import numpy as np
from math import sin, cos

class Transormation():
    def __init__(self, dx, dy, dalpha):
        self.dx = dx
        self.dy = dy
        self.dalpha = dalpha
        self.score = None

    def transform(self, coords, center = None):
        transformed = []
        center = np.sum(coords, axis=0)/len(coords) if center is None else np.array(center)
        rMat = np.array([[cos(self.dalpha), sin(self.dalpha)],
                         [-sin(self.dalpha), cos(self.dalpha)]])
        tMat = np.array([self.dx, self.dy])

        for coord in coords:
            newcoord = np.matmul((coord - center), rMat) + center + tMat
            transformed.append(newcoord)

        return np.array(transformed)


class GeneticSLAM():
    def __init__(self, max_iter = 5, pop_size = 100, pixelSpan = 720, distSpan = 10):
        self.map_ = None
        self.x = 0
        self.y = 0
        self.theta = 0
        self.max_iter = max_iter
        self.pop_size = pop_size
        self.pixelSpan = 720
        self.distSpan = 10

    def scanToCoords(self, scanList):
        coords = []
        scaler = self.pixelSpan/self.distSpan
        for i in range(len(scanList)):
            if scanList[i] != np.inf:
                pt = [int(scaler*scanList[i]*sin((i-90)*np.pi/180)), int(scaler*scanList[i]*cos((i-90)*np.pi/180))]
                coords.append(pt)
        return np.array(coords)

    def score(self, t, coords):
        transformed = t.transform(coords, (self.x, self.y))
        s = 0
        for point in transformed:
             dist_2 = np.sum((self.map_ - point)**2, axis=1)
             dist = np.min(dist_2)**0.5
             s -= dist
        return s

    def match(self, coords):
        epsilon = 1 * (self.pixelSpan/self.distSpan)
        gamma = 0.5 * np.pi
        population = []
        best_transform = Transormation(0,0,0)
        for i in range(self.max_iter):
            pert_range = (-epsilon/2, epsilon/2)
            angle_pert_range = (-gamma/2, gamma/2)
            for p in range(self.pop_size):
                base = np.array([best_transform.dx, best_transform.dy, best_transform.dalpha])
                perturbation = np.array([random.uniform(*pert_range), random.uniform(*pert_range), random.uniform(*angle_pert_range)])
                t = Transormation(*tuple(base+perturbation))
                population.append(t)
                t.score = self.score(t, coords)

            best_transform = max(population, key = lambda x: x.score)
            epsilon /= 2
            gamma /= 2

        self.x += best_transform.dx
        self.y += best_transform.dy
        self.theta += best_transform.dalpha

        return best_transform

    def union(self, coords1, coords2, thres = 10):
        new = []
        for point in coords2:
             dist_2 = np.sum((coords1 - point)**2, axis=1)
             dist = np.min(dist_2)**0.5
             if dist >= thres:
                new.append(point)
        new = np.array(new)

        if len(new) > 0:
            return np.append(coords1, new, axis=0)
        else:
            return coords1

    def update(self, scan):
        print(len(self.map_) if self.map_ is not None else 0)
        precoords = self.scanToCoords(scan)
        ref = Transormation(self.x, self.y, self.theta)
        coords = ref.transform(precoords, (self.x, self.y))

        if self.map_ is None:
            self.map_ = coords
            return self.makeImage(self.map_)
        else:
            t = self.match(coords)
            newpts = t.transform(coords, (self.x, self.y))
            self.map_ = self.union(self.map_, newpts)
            # self.map_ = np.append(self.map_, newpts, axis=0)
            # print(self.map_, newpts)
            return self.makeImage(self.map_)

    def makeImage(self, coords):
        # blank = np.ones((self.pixelSpan, self.pixelSpan))*255
        blank = np.ones((1080,1080))*255
        # blank = np.stack((blank,)*3, axis=-1)
        for pt in self.map_:
            blank = cv2.circle(blank, (int(pt[0]+720), int(pt[1]+720)), 2, 0, -2)

        return blank


if __name__ == '__main__':
    pass
