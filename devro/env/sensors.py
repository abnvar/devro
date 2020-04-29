import cv2
import numpy as np
from math import sin, cos

class Encoder():
    '''
    Class to hold motor encoder properties

    ...

    Attributes
    ----------
    ppr : int
        number of pulses per rotation

    Methods
    -------
    incrementCounter(x)
        increments self.counter by x
    '''

    def __init__(self, ppr=3600):
        self.ppr = ppr
        self.counter = 0

    def incrementCounter(self, x):
        self.counter += x

    def reset(self):
        self.counter = 0


class Scanner():
    '''
    Class to hold scanner(lidar) properties

    ...

    Attributes
    ----------
    ppr : int
        points per revolution (default 360)

    range : int
        maximum detection range of the scanner (in meters) (default 2)

    minDist : float
        minimum detection range of the scanner (in meters) (default 0.01)

    resolution : float
        distance measuring accuracy/resolution (in meters) (default 0.05)

    fieldAngle : float
        angle range covered by the scans (in radians) (default 4.01 or
        230 degrees). If you have multiple scanners then the coverage is 2*pi.

    distScannerToRobotCenter: float
        distance between the scanner and the robot's center (in meters).
        creates a new coordinate system where the center is the scanner's
        position.

    Methods
    -------
    None
    '''

    def __init__(self, ppr=360, range_=2, minDist=0, resolution=0.05,
                 fieldAngle = 4.01,
                 distScannerToRobotCenter = 0.03):

        self.ppr = ppr
        self.range_ = range_
        self.minDist = minDist
        self.resolution = resolution
        self.fieldAngle = fieldAngle
        self.distScannerToRobotCenter = distScannerToRobotCenter

    def attachBot(self, bot):
        self.bot = bot
        self.pixelSpan = self.bot.sim.pixelSpan
        self.distSpan = self.bot.sim.distSpan

    def scan(self, visualize = True):
        # Scaling
        range_ = self.range_ * (self.pixelSpan/self.distSpan)
        minDist = self.minDist * (self.pixelSpan/self.distSpan)
        resolution = self.resolution * (self.pixelSpan/self.distSpan)

        scanList = []
        alpha = self.fieldAngle/self.ppr

        self.start = -self.ppr//2
        self.end = self.ppr//2 if self.ppr % 2 == 0 else self.ppr//2+1

        for k in range(0, self.ppr):
            j, i = self.bot.x, self.bot.y
            i += minDist*cos(-self.bot.theta+alpha*k)
            j += minDist*sin(-self.bot.theta+alpha*k)
            step = 0
            while (0 < i < self.pixelSpan and 0 < j < self.pixelSpan) and minDist + resolution*step < range_ and self.bot.map_[int(i)][int(j)] != 0:
                i += resolution*cos(-self.bot.theta+alpha*k)
                j += resolution*sin(-self.bot.theta+alpha*k)
                step += 1
            if (0 < i < self.pixelSpan and 0 < j < self.pixelSpan) and self.bot.map_[int(i)][int(j)] == 0:
                val = (minDist+resolution*step)*(self.distSpan/self.pixelSpan) if step != 0 else np.inf
                scanList.append(val)
            else:
                scanList.append(np.inf)

        if visualize == True:
            scanImg = self.imagifyScan(scanList)
            self.bot.sim.showScanner(scanImg)

        return scanList

    def imagifyScan(self, scanList):
        blank = np.ones((self.pixelSpan,self.pixelSpan))*255
        scaler = self.pixelSpan/self.distSpan
        blank = cv2.circle(blank, (self.pixelSpan//2, self.pixelSpan//2), 4, (0,0,0), -4)
        for i in range(0, self.ppr):
            if scanList[i] != np.inf:
                center = (self.pixelSpan//2+int(scaler*scanList[i]*sin((i+90)*self.fieldAngle/360)), self.pixelSpan//2+int(scaler*scanList[i]*cos((i+90)*self.fieldAngle/360)))
                blank = cv2.circle(blank, center, 2, (0,0,0), -2)

        return blank
