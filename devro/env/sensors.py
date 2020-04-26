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

    angleField : float
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

    def __init__(self, ppr=360, range_=2, minDist = 0.1, resolution=0.05,
                 angleField = 4.01,
                 distScannerToRobotCenter = 0.03):

        self.ppr = ppr
        self.range_ = range_
        self.minDist = minDist
        self.resolution = resolution
        self.angleField = angleField
        self.distScannerToRobotCenter = distScannerToRobotCenter
