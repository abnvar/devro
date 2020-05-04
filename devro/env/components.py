import cv2
import time
import simpy
import random
import threading
import numpy as np
from math import sin, cos

from devro.visualization import display

class DynObstacle():
    def __init__(self, radius, velocity, pixelSpan, direction):
        self.velocity = velocity
        self.position = {"x":random.randint(0,pixelSpan),"y":random.randint(0,pixelSpan)}
        self.radius = radius
        self.direction = direction

    def updatePosition(self, dt):
        self.position = {"x": self.position["x"]+ self.velocity*cos(self.direction)*dt,
                         "y": self.position["y"]+ self.velocity*sin(self.direction)*dt}

    def switchDirection(self):
        self.velocity = -self.velocity


class Bot():
    '''
    Class to hold the robot properties

    ...

    Attributes
    ----------
    scanner : class
        Adds a scanner object to the robot.

    wheelDist : float
        distance between the wheels center (in meters)

    Methods
    -------
    None
    '''

    def __init__(self, leftMotor, rightMotor, wheelDist, scanner = None):
        self.scanner = scanner
        self.wheelDist = wheelDist
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.vl = 0
        self.vr = 0
        self.theta = 0
        self.omega = 0
        self.vx = 0
        self.vy = 0
        self.map_ = None

    def setInitPos(self, x, y):
        self.x = x
        self.y = y

    def setDestPos(self, x, y):
        self.destX = x
        self.destY = y

    def attachSim(self, sim, envMap):
        self.sim = sim
        self.map_ = envMap

        self.setInitPos(50, sim.pixelSpan - 50)
        self.setDestPos(sim.pixelSpan - 50, 50)
        self.wheelDist = int(self.wheelDist * (sim.pixelSpan/sim.distSpan))

        if self.scanner is not None:
            self.scanner.attachBot(self)

    def updateEncoders(self, dt):
        if self.leftMotor.encoder != None:
            self.leftMotor.updateEncoder(dt)
        if self.rightMotor.encoder != None:
            self.rightMotor.updateEncoder(dt)

    def collisionHandler(self, clearance):
        collision = False
        collDir = 0

        if collision is False:
            numPts = 0
            for phi in range(360):
                pt = (int(self.x + self.wheelDist*cos(phi*np.pi/180)/2), int(self.y + self.wheelDist*sin(phi*np.pi/180)/2))
                val = self.map_[pt[1]][pt[0]]
                # if black
                if val < 127:
                    collision = True
                    collDir += phi
                    numPts += 1

            collDir = collDir/numPts if numPts != 0 else 0   # average of the direction of pixels of obstacles

            if abs(collDir%360 - (180/np.pi)*self.theta%360) > 90:
                collision = False

        return collision, collDir

    def drive(self, dt):
        clearance = self.wheelDist/2

        collision, collDir = self.collisionHandler(clearance)
        # self.sim.active = (self.x > clearance and self.y > clearance and self.x < self.sim.pixelSpan-clearance and self.y < self.sim.pixelSpan-clearance)
        v = (self.vl + self.vr)/2
        if collision is True:
            phi = (collDir+90)*np.pi/180
            self.vx = v*cos(self.theta-phi)*cos(phi)
            self.vy = v*cos(self.theta-phi)*sin(phi)
        else:
            self.vx = v*cos(self.theta)
            self.vy = v*sin(self.theta)

        if (self.x + self.vx*dt > clearance) and (self.x + self.vx*dt < self.sim.pixelSpan-clearance):
            self.x += self.vx*dt
        if (self.y + self.vy*dt > clearance) and (self.y + self.vy*dt < self.sim.pixelSpan-clearance):
            self.y += self.vy*dt
        self.omega = (self.vl-self.vr)/(self.wheelDist)
        self.theta += self.omega*dt

        self.updateEncoders(dt)

    def setVel(self, vl, vr):
        self.leftMotor.setSpeed(vl)
        self.rightMotor.setSpeed(vr)
        self.vl = self.leftMotor.groundVelocity() * (self.sim.pixelSpan/self.sim.distSpan)
        self.vr = self.rightMotor.groundVelocity() * (self.sim.pixelSpan/self.sim.distSpan)

    def plotBot(self, canvas):
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), self.wheelDist//2, (0,0,0), -self.wheelDist//2)    # Robot
        canvas = cv2.putText(canvas, 'x', (self.destX, self.destY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)     # Dest cross
        canvas = cv2.line(canvas, (int(self.x), int(self.y)), (int(self.x+self.wheelDist*cos(self.theta)), int(self.y+self.wheelDist*sin(self.theta))), (0,0,0), 2)    # direction line
        if self.scanner is not None:
            angle = self.scanner.fieldAngle*180/np.pi
            rangeRad = int(self.scanner.range_*(self.sim.pixelSpan/self.sim.distSpan))
            canvas = cv2.ellipse(canvas, (int(self.x), int(self.y)), (rangeRad, rangeRad), (180/np.pi)*self.theta-angle//2, 0, angle, (255,0,0), 2)     # scanner range
            minRad = int(self.scanner.minDist*(self.sim.pixelSpan/self.sim.distSpan))
            canvas = cv2.ellipse(canvas, (int(self.x), int(self.y)), (minRad, minRad), (180/np.pi)*self.theta-angle//2, 0, angle, (127,0,127), 2)   # scanner minDist

        return canvas

    def updateMap(self, img):
        self.map_ = img

    def reset(self):
        self.theta = 0
        self.omega = 0
        self.setVel(0, 0)
        self.setInitPos(50, self.sim.pixelSpan - 50)


    def scannerToWorld(self,pose, point):

        """ Given a robot pose (xR, yR, thetaR) and a point (xL, yL) from a
        landmark in the scanner's coordinate system, return the point's
        coordinates in the world coordinate system. The point could be a
        landmark, or any point measured the scanner sensor.

        Args:
        pose (tuple): Robot's pose.
        point (tuple): The point to transform to world coordinates.

        Returns:
        tuple: The transformed point to the world coordinate system.

        """

        scannerPose = ( pose[0] + cos(pose[2]) * self.scanner.distToRobotCenter,
                    pose[1] + sin(pose[2]) * self.scanner.distToRobotCenter,
                    pose[2] )

        x, y = point

        # check rotation matrix
        return (x * cos(scannerPose[2]) - y * sin(scannerPose[2]) + scannerPose[0],
                x * sin(scannerPose[2]) + y * cos(scannerPose[2]) + scannerPose[1])


class Simulation(threading.Thread):
    '''
    The head simulation class to hold the simulation parameters.

    ...

    Attributes
    ----------
    pixelSpan : int
        number of pixels in one row of the simulation screen (sim is always square)

    distSpan : int
        distance (in meters) in actual world corresponding to each row of the simulation screen

    dt : int
        one time step of simulation (in miliseconds)

    envMap : numpy image array (pixelSpan x pixelSpan)
        Map of the simulation world

    bot : class
        object of the Bot class

    visualize : bool
        whether to visualize the simulation or not

    Methods
    -------
    begin()
        starts the simpy simulation

    showSimulation()
        updates the environment frame on the display component

    showScanner()
        updates the scanner output fram on the display component
    '''

    def __init__(self, pixelSpan = 720, distSpan = 10, dt = 100, envMap = None, bot = None, visualize = True):
        threading.Thread.__init__(self)
        self.daemon = True
        self.pixelSpan = pixelSpan
        self.distSpan = distSpan
        self.dt = dt/1000   # converting to milliseconds
        self.envMap = envMap
        self.currMap = envMap
        self.bot = bot
        self.visualize = visualize
        self.env = simpy.RealtimeEnvironment(strict=False)
        self.active = True
        self.paused = False
        self.obstacles = []
        self.landmarks = None

        if self.visualize is True:
            self.win = display.Window('Simulation', height = self.pixelSpan, dt = dt, endSimFunc = self.end, toggleSimFunc = self.toggle, scale = 0.7)

        bot.attachSim(self, self.currMap)
        self.stepProc = self.env.process(self.step(self.env))

    def step(self, env):
        while self.active:
            if self.paused is False:
                self.updateObstacles()
                self.bot.drive(self.dt)

            if self.visualize == True:
                self.showEnv()
                self.plotLandmarks()

            yield env.timeout(self.dt)

    def toggle(self):
        if self.paused is True:
            self.paused = False
        else:
            self.paused = True

    def run(self):
        self.env.run(until=self.stepProc)
        self._is_running = False

    def hold(self):
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                self.end()

    def begin(self):
        self.start()

    def end(self):
        try:
            self.win.close()
        except:
            pass
        self.active = False
        import os
        os._exit(0)

    def addDynamicObstacles(self, qty=0, radiusRange=(10,20), maxVelocity=10):
        minR, maxR = radiusRange

        for x in range(qty):
            self.obstacles.append(DynObstacle(radius = random.randint(minR, maxR),
                                              velocity = random.randint(-maxVelocity, maxVelocity),
                                              pixelSpan = self.pixelSpan,
                                              direction = random.uniform(0, 2*np.pi)
                                              ))

    def updateObstacles(self):
        mask = np.asarray(self.envMap, np.uint8)
        for obstacle in self.obstacles:
            obstacle.updatePosition(self.dt)
            if not 0 < int(obstacle.position["x"]) < self.pixelSpan or not 0 < int(obstacle.position["y"]) < self.pixelSpan:
                obstacle.switchDirection()
            else:
                mask = cv2.circle(mask, (int(obstacle.position["x"]), int(obstacle.position["y"])), obstacle.radius, (0,0,0), -obstacle.radius)

        self.currMap = mask
        self.bot.updateMap(self.currMap)

    def reset(self):
        self.bot.reset()
        self.bot.leftMotor.encoder.reset()
        self.bot.rightMotor.encoder.reset()

    def showEnv(self):
        canvas = np.asarray(self.currMap, np.uint8)
        canvas = np.stack((canvas,)*3, axis=-1)
        canvas = self.bot.plotBot(canvas)
        self.win.setEnvFrame(canvas)

    def showScanner(self, img):
        self.win.setScannerFrame(img)

    def setLandmarks(self, pts):
        self.landmarks = pts

    def plotLandmarks(self):
        if self.landmarks is not None:
            mask = np.ones((self.pixelSpan, self.pixelSpan))*255
            pts = np.asarray(self.landmarks).T
            scaler = (self.pixelSpan/self.distSpan)
            for x,y in pts:
                mask = cv2.circle(mask, (int(x*scaler), int(y*scaler)), 2, (0,0,0), -2)
            self.win.setSlamFrame(mask)
