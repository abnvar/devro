import time
import simpy
import cv2
import threading
import numpy as np
from math import sin, cos
import random
from devro.visualization import display

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

    def __init__(self, leftMotor, rightMotor, scanner, wheelDist):
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
        self.x = None
        self.y = None
        self.destX = None
        self.destY = None
        self.sim = None
        self.env = None
        self.map_ = None
        self.obstacleMap_ = None
        self.completeMap_ = None


    def setInitPos(self, x, y):
        self.x = x
        self.y = y

    def setDestPos(self, x, y):
        self.destX = x
        self.destY = y

    def attachSim(self, sim, envMap):
        self.sim = sim
        self.map_ = envMap
        self.completeMap_ = envMap

        self.setInitPos(50, sim.pixelSpan - 50)
        self.setDestPos(sim.pixelSpan - 50, 50)
        self.wheelDist = int(self.wheelDist * (sim.pixelSpan/sim.distSpan))

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
                val = self.completeMap_[pt[1]][pt[0]]
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

    def scan(self, visualize = True):
        # Scaling
        range_ = self.scanner.range_ * (self.sim.pixelSpan/self.sim.distSpan)
        minDist = self.scanner.minDist * (self.sim.pixelSpan/self.sim.distSpan)
        resolution = self.scanner.resolution * (self.sim.pixelSpan/self.sim.distSpan)

        scanList = []
        alpha = 2*np.pi/self.scanner.ppr

        for k in range(self.scanner.ppr):
            j, i = self.x, self.y
            step = 0
            while (i > 0 and i < self.sim.pixelSpan and j > 0 and j < self.sim.pixelSpan) and resolution*step < range_ and self.completeMap_[int(i)][int(j)] != 0:
                i += resolution*cos(np.pi-self.theta+alpha*k)
                j += resolution*sin(np.pi-self.theta+alpha*k)
                step += 1
            if (i > 0 and i < self.sim.pixelSpan and j > 0 and j < self.sim.pixelSpan) and self.completeMap_[int(i)][int(j)] == 0:
                scanList.append(resolution*step *(self.sim.distSpan/self.sim.pixelSpan))
            else:
                scanList.append(np.inf)

        if visualize == True:
            scanImg = self.imagifyScan(scanList)
            self.sim.showScanner(scanImg)

        return scanList

    def imagifyScan(self, scanList):
        blank = np.ones((self.sim.pixelSpan,self.sim.pixelSpan))*255
        scaler = self.sim.pixelSpan/self.sim.distSpan
        blank = cv2.circle(blank, (self.sim.pixelSpan//2, self.sim.pixelSpan//2), 4, (0,0,0), -4)
        for i in range(self.scanner.ppr):
            if scanList[i] != np.inf:
                center = (self.sim.pixelSpan//2+int(scaler*scanList[i]*sin((i-90)*np.pi/180)), self.sim.pixelSpan//2+int(scaler*scanList[i]*cos((i-90)*np.pi/180)))
                blank = cv2.circle(blank, center, 2, (0,0,0), -2)

        return blank

    def plotBot(self, canvas):
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), self.wheelDist//2, (0,0,0), -self.wheelDist//2)    # Robot
        canvas = cv2.putText(canvas, 'x', (self.destX, self.destY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        canvas = cv2.circle(canvas, (int(self.x), int(self.y)), int(self.scanner.range_*(self.sim.pixelSpan/self.sim.distSpan)), (255,0,0), 2)      # scanner circle
        canvas = cv2.line(canvas, (int(self.x), int(self.y)), (int(self.x+self.wheelDist*cos(self.theta)), int(self.y+self.wheelDist*sin(self.theta))), (0,0,0), 2)    # direction line

        return canvas

    def updateObstacleMap(self,image):
        self.obstacleMap_ = image
        self.completeMap_ = cv2.bitwise_and(self.map_.astype(np.uint8),self.obstacleMap_.astype(np.uint8))



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

    def __init__(self, pixelSpan = 720, distSpan = 10, dt = 100, envMap = None, bot = None, visualize = True , dynamicObstacles=0):
        threading.Thread.__init__(self)
        self.daemon = True
        self.pixelSpan = pixelSpan
        self.distSpan = distSpan
        self.dt = dt/1000   # converting to milliseconds
        self.envMap = envMap
        self.bot = bot
        self.visualize = visualize
        self.obstacleMap = None

        self.obstacles = [
        ]

        for x in range(dynamicObstacles):
            self.obstacles.append({
                "velocity": random.randint(-6,6),
                "position":{"x":random.randint(0,pixelSpan),"y":random.randint(0,pixelSpan)}
                })


        self.env = simpy.RealtimeEnvironment(strict=False)
        self.active = True
        self.paused = False

        if self.visualize is True:
            self.win = display.Window('Simulation', height = self.pixelSpan, dt = dt, endSimFunc = self.end, toggleSimFunc = self.toggle, scale = 0.7)

        bot.attachSim(self, self.envMap)
        self.stepProc = self.env.process(self.step(self.env))

    def updateObstacles(self):
        mask = np.zeros(self.envMap.shape, np.uint8)
        for i in range(len(self.obstacles)):
            self.obstacles[i]["position"] =  {"x":self.obstacles[i]["position"]["x"]+ self.obstacles[i]["velocity"]*self.dt ,"y":self.obstacles[i]["position"]["y"]+ self.obstacles[i]["velocity"]*self.dt}
            if(int(self.obstacles[i]["position"]["x"]) >= self.pixelSpan or int(self.obstacles[i]["position"]["x"]) <= 0 or int(self.obstacles[i]["position"]["y"]) >= self.pixelSpan or int(self.obstacles[i]["position"]["y"]) <= 0 ):
                self.obstacles[i]["velocity"] = -self.obstacles[i]["velocity"] 
            else:
                mask = cv2.circle(mask, (int(self.obstacles[i]["position"]["x"]), int(self.obstacles[i]["position"]["y"])),15, (255, 255, 255), 25) 

        self.obstacleMap = np.invert(mask)
        self.bot.updateObstacleMap(self.obstacleMap)




    def step(self, env):
        while self.active:
            if self.paused is False:
                self.updateObstacles()

                self.bot.drive(self.dt)

            if self.visualize == True:
                self.showEnv()

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



    def reset(self):
        self.bot.reset()
        self.bot.leftMotor.encoder.reset()
        self.bot.rightMotor.encoder.reset()

    def showEnv(self):
        obstacleMap_ = cv2.bitwise_and(self.envMap.astype(np.uint8),self.obstacleMap.astype(np.uint8))
        canvas = np.stack((obstacleMap_,)*3, axis=-1)
        canvas = self.bot.plotBot(canvas)
        self.win.setEnvFrame(canvas)

    def showScanner(self, img):
        self.win.setScannerFrame(img)
