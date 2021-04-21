import pilot
import imgProcess as img
import frontier
import numpy as np

#Class responsible for navigation
class Navigator():
    def __init__(self, robot, targetColor):
        self.pilot = pilot.Pilot(robot)
        self.cones = []
        self.target = targetColor
        self.path = []
        self.toppledCones=[]
        self.goal = None
        self.stateDistance = 0.2
        self.keepConeDistance = 0.4
        self.acceptedPathLenght = 5
        self.explored = []
        self.kRGB = self.pilot.getRGBKmatrix()
        self.maxSearchIterations = 3

        self.pilot.resetPosition()
        print("INFO: Navigator is now initialized")

    def toppleTargetCone(self):
        print("INFO: Cone will be toppled")
        reached = False
        check = True
        while (not reached):
            self.setGoal()
            self.findPath(self.goal)
            if (not self.path):
                print("ERROR: No path found")
                break
            if (len(self.path) > self.acceptedPathLenght+3):
                print("INFO: Getting closer to the goal")
                self.pilot.drive(self.path[:self.acceptedPathLenght], False) # odometry reset
                self.updateToppledCones(self.pilot.getCurrentPos())
                check = self.scanForCones(0)
            else:
                print("INFO: Cone is close enough, driving straight to it")
                self.pilot.drive(self.path, True) # odometry reset
                self.updateToppledCones(self.pilot.getCurrentPos())
                reached = True
                self.toppledCones.append(self.goal)
                print("INFO: Cone toppled")
            if (not check):
                print("INFO: Lost target")
                break
        self.goal = None

    def setGoal(self):
        targetCone = None
        for x in range(len(self.cones)):
            if (self.cones[x].color == self.target and self.cones[x].standing):
                targetCone = (self.cones[x].coord.x, self.cones[x].coord.y)
                break
        self.goal = targetCone

    def findPath(self, goal):
        print("INFO: Searching for optimal path")
        queue = frontier.queue(goal)  # initialize the frontier
        queue.push([[(0, 0), 0]], [None, 0, None, 0])  # push the first node into the frontier
        self.explored = []
        i = 0
        while i < 10000:
            current = queue.pop()

            if (current is None):  # what to do if the frontier is empty
                self.path = None
                break
            elif (self.isCloseToCone(current[0], goal, False, False)):  # reached the end state
                self.extractPath(current, goal)
                break

            children = self.getNextStates(current[0])  # uncover the current nodes children [[(x1, y1), cost], [(x2, y2), cost], ... ]
            self.explored.append(current[0])
            children = self.removeExpolored(children)
            queue.push(children, current)
            i += 1

        if i < 10000:
            print("INFO: Optimal path found!")
        else:
            self.path = None
            print("INFO: Path not found, scanning for a new target.")

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
    def extractPath(self, node, goal):
        path = []
        oldest = node[0]

        while (not (oldest[0] == 0 and oldest[1] == 0)):
            path.append(oldest)
            node = node[2]
            oldest = node[0]

        path.reverse()
        path.append(goal)
        self.path = path

    def scanForCones(self, depth):
        angle = np.pi / 8
        maxRotations = 15
        someCones = self.rotatingScan(angle, maxRotations)

        maxSearch = self.maxSearchIterations
        if (self.toppledCones == []):
            maxSearch = 5

        if (someCones[0] == self.target):
            return True
        elif (someCones[0] == "Some" and depth < maxSearch):
            print("INFO: No target Cones found. Changing vintage point.")
            self.pilot.rotateByAngle((someCones[1]+1)*angle) # odometry reset
            self.updateToppledCones(self.pilot.getCurrentPos())
            self.getConesFromImage()
            check = self.validateVintagePoint()
            numrot = 0
            while ((not check) and numrot < maxRotations):
                self.pilot.rotateByAngle(angle)  # odometry reset
                self.updateToppledCones(self.pilot.getCurrentPos())
                self.getConesFromImage()
                check = self.validateVintagePoint()
            if (check):
                self.goal = (1, 0)
                self.findPath(self.goal)
                self.pilot.drive(self.path, False) # odometry reset
                self.updateToppledCones(self.pilot.getCurrentPos())
                return self.scanForCones(depth+1)
            else:
                print("INFO: Unable to get to a different vintage point")
                return False
        else:
            return False

    def rotatingScan(self, angle, maxRotations):
        print("INFO: Starting rotation")
        iteration = 0
        someCones = ("None", -1)
        while (iteration < maxRotations):
            self.cones = []
            self.getConesFromImage()
            if (self.cones != []):
                if (someCones[0] == "None"):
                    someCones = ("Some", iteration)
                if (self.targetConeFound()):
                    print("INFO: Target cone found")
                    someCones = (self.target, iteration)
                    return someCones
            print("INFO: No target cones found, rotating.")
            print("")
            self.pilot.rotateByAngle(angle) # odometry reset
            self.updateToppledCones(self.pilot.getCurrentPos())
            iteration += 1
        return someCones

    def validateVintagePoint(self):
        check = True
        newPoint = (1, 0)
        for cone in self.cones:
            if (not np.isnan(cone.coord.x) and not np.isnan(cone.coord.y) and not np.isnan(cone.coord.z)):
                if(self.isCloseToCone(newPoint, (cone.coord.x, cone.coord.y), False, True)):
                    check = False
                    break
        return check

    def targetConeFound(self):
        found = False
        for cone in self.cones:
            if (cone.color == self.target and cone.standing):
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
        i = 0
        deleted = 0
        while (i <len(self.cones)):
            if (np.isnan(self.cones[i-deleted].coord.x) or np.isnan(self.cones[i-deleted].coord.y) or np.isnan(self.cones[i-deleted].coord.z)):
                self.cones.remove(self.cones[i-deleted])
                deleted += 1
            i += 1
        for cone in self.cones:
            if cone.color == self.target and cone.standing:
                for toppled in self.toppledCones:
                    if (img.distance(toppled, cone) <= 1):
                        print("INFO: I see a target cone that i already toppled.")
                        #cone.standing = False #Removing toppled cones not working - commented out to prevent unexpected behavior
                        break

        print("Scanning complete")

    def updateToppledCones(self, newPosition):
        for x in range(len(self.toppledCones)):
            X = self.toppledCones[x][0] - newPosition[0]
            Y = self.toppledCones[x][1] - newPosition[1]
            X = X*np.cos(newPosition[2])-Y*np.sin(newPosition[2])
            Y = -X * np.sin(newPosition[2]) + Y * np.cos(newPosition[2])
            self.toppledCones[x] = (X, Y)