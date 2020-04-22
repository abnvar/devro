import numpy as np

class Motor():
    '''
    Class to hold motor properties

    ...

    Attributes
    ----------
    mode : str
        unit of velocity input
        Valid values:
            'rpm', 'radpersec', 'groundvelocity'

    maxSpeed : float
        Maximum allowed velocity of the motor (in the specified mode units)

    wheelRadius : float
        radius of the wheels (in meters)

    Methods
    -------
    setSpeed(x)
        set motor speed to x

    groundvelocity()
        get velocity of the point of wheel that touches the ground / velocity of motor relative to ground (in meters/sec)

    updateEncoder()
        updates the value of the encoder attached to the motor
    '''
    def __init__(self, mode='rpm', maxSpeed = np.inf, wheelRadius = None, encoder = None):
        self.mode = mode
        self.speed = 0
        self.maxSpeed = maxSpeed
        self.encoder = encoder
        if wheelRadius != None:
            self.wheelRadius = wheelRadius
        else:
            pass    # motors such as drone motors do not have this property,
                    # hence that case can be handled here later.

        if mode == 'rpm':
            self.converter = lambda x : min(x, maxSpeed) * (2 * np.pi / 60) * wheelRadius
            self.encFactor = 1/60
        elif mode == 'radpersec':
            self.converter = lambda x : min(x, maxSpeed) * wheelRadius
            self.encFactor = 1/(2*np.pi)
        elif mode == 'groundvelocity':
            self.converter = lambda x : min(x, maxSpeed)
            self.encFactor = 1/(2*np.pi*self.wheelRadius) if self.wheelRadius != None else None
        else:
            raise Exception("Invalid value of mode. Valid values are 'rpm' / 'radpersec' / 'groundvelocity'")

    def setSpeed(self, x):
        self.speed = x

    def groundVelocity(self):
        return self.converter(self.speed)

    def updateEncoder(self, dt):
        self.encoder.incrementCounter(self.encoder.ppr*self.encFactor*self.speed*dt)
