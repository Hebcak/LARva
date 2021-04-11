from robolab_turtlebot import Rate, get_time
import numpy as np

#class responsible for driving the robot along a specified path
class Pilot():
    def __init__(self, robot):
        self.robot = robot
        self.rate = Rate(10)
        self.start = True
        print("INFO: Pilot has been initialized")

    def drive(self, path, topple):
        print(path)
        self.resetPosition()
        for x in range(len(path)):
            angle = self.getBearing(path[x])
            self.setBearing(angle)
            self.driveTo(path[x], (x == len(path)-1) and topple)

#drives to point in a straight line
    def driveTo(self, point, topple):
        if topple:
            v = 1
        else:
            v = 0.2
        prevDist = float('Inf')
        distance = self.getDistance(point)
        while (distance > 0.02 and distance < prevDist):
            self.robot.cmd_velocity(linear=v)
            prevDist = distance
            self.rate.sleep()
            distance = self.getDistance(point)
        if not (distance < 0.05):
            print("INFO: Adjustment needed to reach: ", point)
            angle = self.getBearing(point)
            self.setBearing(angle)
            self.driveTo(point, False)

    def getDistance(self, point):
        currentPos = self.getCurrentPos()
        dx = point[0] - currentPos[0]
        dy = point[1] - currentPos[1]
        distance = ((dx ** 2) + (dy ** 2)) ** 0.5
        return distance

#rotates the robot angle radians
    def setBearing(self, angle):
        curAngle = self.getCurrentPos()[2]
        self.start = False
        da = angle - curAngle
        v = np.sign(da)*0.1
        angleInt = angle
        while (v > 0 and curAngle < angleInt) or (v < 0 and curAngle > angleInt):
            self.robot.cmd_velocity(angular=v)
            self.rate.sleep()
            curAngle = self.getCurrentPos()[2]

    def getBearing(self, point):
        currentPos = self.getCurrentPos()
        dx = point[0] - currentPos[0]
        dy = point[1] - currentPos[1]
        if (dx != 0):
            angle = np.arctan(dy/dx)
        else:
            angle = np.pi/2
        if (dx < 0 ):
            angle += np.pi
        if (angle > np.pi):
            angle -=2*np.pi
        return angle

    #Fix the skew of the robot for testing
    def rotate2zero(self):
        if self.start:
            t = get_time()
            v = -0.14
            while ((get_time() - t) < 1):
                self.robot.cmd_velocity(angular=v)
                self.rate.sleep()
            self.robot.reset_odometry()
            print("INFO: The angle is now true 0")
        else:
            print("ERROR: Rotate2zero function can be called only on startup")

    def rotateRelatively(self, angle):
        curAngle = self.getCurrentPos()[2]
        self.start = False
        finalAngle = curAngle + angle
        # maybe angle returned by get_odometry is in range [0, 2pi]? Ask Hynek
        # if (finalAngle > 2*np.pi): finalAngle -= 2*np.pi
        # elif (finalAngle < 0): finalAngle += 2*np.pi
        v = np.sign(angle) * 0.1
        while (v > 0 and curAngle < finalAngle) or (v < 0 and curAngle > finalAngle):
            self.robot.cmd_velocity(angular=v)
            self.rate.sleep()
            curAngle = self.getCurrentPos()[2]

    def getCurrentPos(self):
        odometry = self.robot.get_odometry()
        if (self.start or odometry is None):
            odometry = [0, 0, 0]
        return odometry

    def getRGBKmatrix(self):
        try:
            kRGB = self.robot.get_rgb_K()
        except:
            kRGB = np.array([[554.25469119, 0, 320.5], [0, 554.25469119, 240.5], [0, 0, 1]])
        return kRGB
    def resetPosition(self):
        self.robot.reset_odometry()
        self.start = True