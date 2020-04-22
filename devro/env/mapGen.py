'''
    Generates a random map using random perlin noise.
    Call getRandomMap() without any parameters to obtain a map as a 2d numpy array.
'''

import noise
import numpy as np
from random import randint

from devro.config.envconfig import envdata

def getRandomMap(pixelSpan = 720, distSpan = 10, randbase = None):
    shape = (envdata.pixelSpan, envdata.pixelSpan)
    scale = 2*envdata.pixelSpan/envdata.distSpan
    octaves = 1
    persistence = 0.5
    lacunarity = 2.0

    if randbase == None:
        randbase = randint(0,1000)
    threshold = 0.25

    noiseMap = np.zeros(shape)
    for i in range(shape[0]):
        for j in range(shape[1]):
            noiseMap[i][j] = noise.pnoise2(i/scale,
                                        j/scale,
                                        octaves=octaves,
                                        persistence=persistence,
                                        lacunarity=lacunarity,
                                        repeatx=1024,
                                        repeaty=1024,
                                        base=randbase)

    genMap = np.zeros(shape)

    for i in range(shape[0]):
        for j in range(shape[1]):
            dist = ((i-shape[1]//2)**2+(j-shape[0]//2)**2)**0.5/(shape[0]/2**0.5)
            if dist < 0.707 and noiseMap[i][j] < threshold:
                genMap[i][j] = 255
            elif dist >=0.707:
                genMap[i][j] = 255

    return genMap


if __name__ == '__main__':
    import cv2
    genMap = getRandomMap()
    cv2.imshow('win', genMap)
    cv2.waitKey(0)
