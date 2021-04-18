import pilot
import img_process as img
import frontier
import numpy as np
import Cone
import Coordinates as xyz

#Class responsible for navigation
class Navigator():
    def __init__(self, robot):
        self.pilot = pilot.Pilot(robot)
        self.cones = []
        self.path = []
        self.toppledCones=[]
        self.goal = None
        self.stateDistance = 0.2
        self.keepConeDistance = 0.3
        self.acceptedPathLenght = 5
        self.explored = []
        self.kRGB = self.pilot.getRGBKmatrix()
        self.maxSearchIterations = 4

        self.pilot.resetPosition()
        print("INFO: Navigator is now initialized")

    def toppleRedCone(self):
        print("INFO: Cone will be toppled")
        reached = False
        lastPos = (0, 0)
        check = True
        while (not reached):
            self.setGoal(lastPos)
            self.findPath(self.goal)
            if (len(self.path) > self.acceptedPathLenght+3):
                print("INFO: Getting closer to the goal")
                lastPos = self.path[self.acceptedPathLenght]
                print("INFO: I will end up at: ", lastPos)
                self.pilot.drive(self.path[:self.acceptedPathLenght], False)
                check = self.scanForCones(0)
            else:
                print("INFO: Cone is close enough, driving straight to it")
                self.pilot.drive(self.path, True)
                reached = True
                self.toppledCones.append(self.goal)
                print("INFO: Cone toppled")
            if (not check):
                print("ERROR: Lost target")
                break
        self.goal = None

    def setGoal(self, lastPos):
        redCone = None
        for x in range(len(self.cones)):
            if (self.cones[x].color == 'Red'):
                redCone = (self.cones[x].coord.x, self.cones[x].coord.y)
                break
        if (redCone is None):
            redCone = self.newGoalFromOld(self.goal, lastPos)
            print("INFO: Goal is invisible, using old goal: ", self.goal, "New coords: ", redCone)
        self.goal = redCone

    def findPath(self, goal):
        print("INFO: Searching for optimal path")
        queue = frontier.queue(goal)  # initialize the frontier
        queue.push([[(0, 0), 0]], [None, 0, None, 0])  # push the first node into the frontier
        self.explored = []

        while True:
            current = queue.pop()

            if (current is None):  # what to do if the frontier is empty
                self.path = None
                break
            elif (self.isCloseToCone(current[0], goal, False, False)):  # reached the end state
                self.extractPath(current)
                break

            children = self.getNextStates(current[0])  # uncover the current nodes children [[(x1, y1), cost], [(x2, y2), cost], ... ]
            self.explored.append(current[0])
            children = self.removeExpolored(children)
            queue.push(children, current)
        print("INFO: Optimal path found!")

    def getNextStates(self, state):
        new = []
        new.append([(state[0] + self.stateDistance, state[1]), self.stateDistance])
        new.append([(state[0] - self.stateDistance, state[1]),self.stateDistance])
        new.append([(state[0], state[1] + self.stateDistance),self.stateDistance])
        new.append([(state[0], state[1] - self.stateDistance),self.stateDistance])
        diagDist = (2*(self.stateDistance**2))**0.5
        new.append([(state[0] + self.stateDistance, state[1] + self.stateDistance), diagDist])
        new.append([(state[0] + self.stateDistance, state[1] - self.stateDistance), diagDist])
        new.append([(state[0] - self.stateDistance, state[1] + self.stateDistance), diagDist])
        new.append([(state[0] - self.stateDistance, state[1] - self.stateDistance), diagDist])
        self.removeBlockedStates(new)
        return new

    def removeBlockedStates(self, states):
        x = 0
        remNum = 0
        while (x < len(states)):
            rem = False
            for cone in self.cones:
                if (self.isCloseToCone(states[x][0], (cone.coord.x, cone.coord.y), True, False)):
                    states.remove(states[x])
                    remNum += 1
                    rem = True
                    break
            if not rem:
                x += 1

    def isCloseToCone(self, state, cone, excludeGoal, scanning):
        dist = (((state[0]-cone[0])**2)+((state[1]-cone[1])**2))**0.5
        keepDist = self.keepConeDistance
        if (scanning):
            keepDist = 0.5
        if (excludeGoal):
            return dist <= keepDist and not (cone[0] == self.goal[0] and cone[1] == self.goal[1])
        else:
            return dist <= keepDist

    # remove already explored nodes from addition to the frontier
    def removeExpolored(self, children):
        correctionShift = 0
        for x in range(len(children)):
            child = children[x-correctionShift]
            if (child[0] in self.explored):
                children.remove(child)
                correctionShift += 1
        return children

    # generate path to node in the proper format
    def extractPath(self, node):
        path = []
        oldest = node[0]

        while (not (oldest[0] == 0 and oldest[1] == 0)):
            path.append(oldest)
            node = node[2]
            oldest = node[0]

        path.reverse()
        path.append(self.goal)
        self.path = path

    def reacquireGoal(self, oldGoal, lastPos):
        print("INFO: Reacquiring goal")
        newGoal = self.newGoalFromOld(oldGoal, lastPos)
        bearing = self.pilot.getBearing(newGoal)
        self.pilot.setBearing(bearing)
        self.getConesFromImage()

    def newGoalFromOld(self, oldGoal, lastPos):
        redCone = (oldGoal[0] - lastPos[0], oldGoal[1] - lastPos[1])
        return redCone

    def scanForCones(self, depth):
        angle = np.pi / 4
        maxRotations = 7
        someCones = self.rotatingScan(angle, maxRotations)
        if (someCones[0] == "Red"):
            return True
        elif (someCones[0] == "Some" and depth < self.maxSearchIterations):
            print("INFO: No red Cones found. Changing vintage point.")
            self.pilot.rotateByAngle((someCones[1]+1)*angle)
            self.getConesFromImage()
            self.goal = self.getNewVintagePoint()
            self.findPath(self.goal)
            self.pilot.drive(self.path, False)
            #self.pilot.rotateByAngle(np.pi/2)
            return self.scanForCones(depth+1)
        else:
            return False

    def rotatingScan(self, angle, maxRotations):
        print("INFO: Scanning for cones")
        iteration = 0
        someCones = ("None", -1)
        while (iteration < maxRotations):
            self.cones = []
            self.getConesFromImage()
            if (self.cones != []):
                if (someCones[0] == "None"):
                    someCones = ("Some", iteration)
                if (self.redConeFound()):
                    print("INFO: Red cone found")
                    someCones = ("Red", iteration)
                    return someCones
            print("INFO: No red cones found, rotating.")
            print("")
            self.pilot.rotateByAngle(angle)
            iteration += 1
        return someCones

    def getNewVintagePoint(self):
        check = False
        newPoint = (1, 0)
        while (not check):
            check = True
            for cone in self.cones:
                if(self.isCloseToCone((1, 0), (cone.coord.x, cone.coord.y), False, True)):
                    check = False
                    break
            if (not check):
                newPoint = (newPoint[0], newPoint[1]+self.stateDistance)
        return newPoint

    def redConeFound(self):
        found = False
        for cone in self.cones:
            if (cone.color == 'Red' and cone.standing):
                found = True
                break
        return found

    def getConesFromImage(self):
        self.pilot.resetPosition()
        image = np.zeros(0)
        depth = np.zeros(0)
        print("INFO: Scanning image for cones... ")
        while (image is None or depth is None or image.size == 0 or depth.size == 0):
            image = self.pilot.robot.get_rgb_image()
            depth = self.pilot.robot.get_depth_image()
        self.cones = img.process(image, depth, self.kRGB)
        print("Scanning complete")
