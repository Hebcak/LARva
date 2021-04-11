import pilot
import img_process as img
import frontier
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

        self.pilot.rotate2zero() #only for testing
        self.pilot.resetPosition()
        print("INFO: Navigator is now initialized")

    def toppleRedCone(self):
        reached = False
        lastPos = (0, 0)
        while (not reached):
            self.setGoal(lastPos)
            self.findPath(self.goal)
            if (len(self.path) > self.acceptedPathLenght+2):
                print("Driving only partial path")
                lastPos = self.path[self.acceptedPathLenght]
                print(lastPos)
                self.pilot.drive(self.path[:self.acceptedPathLenght], False)
                self.scanForCones()
            else:
                print("Driving to the goal")
                self.pilot.drive(self.path, True)
                reached = True
        self.toppledCones.append(self.goal)
        self.goal = None
        print("Cone toppled")

    def setGoal(self, lastPos):
        redCone = None
        for x in range(len(self.cones)):
            if (self.cones[x].color == 'Red'):
                redCone = (self.cones[x].coord.x, self.cones[x].coord.y)
                break
        if (redCone is None):
            redCone = self.goal # don't forget to add coordinate transform
            redCone[0] -= lastPos[0]
            redCone[1] -= lastPos[1]
            print("Goal is invisible, using old goal: ", self.goal, "New coords: ", redCone)
        self.goal = redCone

    def findPath(self, goal):
        queue = frontier.queue(goal)  # initialize the frontier
        queue.push([[(0, 0), 0]], [None, 0, None, 0])  # push the first node into the frontier
        self.explored = []

        while True:
            current = queue.pop()

            if (current is None):  # what to do if the frontier is empty
                self.path = None
                break
            elif (self.isCloseToCone(current[0], goal, False)):  # reached the end state
                self.extractPath(current)
                break

            children = self.getNextStates(current[0])  # uncover the current nodes children [[(x1, y1), cost], [(x2, y2), cost], ... ]
            self.explored.append(current[0])
            children = self.removeExpolored(children)
            queue.push(children, current)

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
                if (self.isCloseToCone(states[x][0], (cone.coord.x, cone.coord.y), True)):
                    states.remove(states[x])
                    remNum += 1
                    rem = True
                    break
            if not rem:
                x += 1

    def isCloseToCone(self, state, cone, excludeGoal):
        dist = (((state[0]-cone[0])**2)+((state[1]-cone[1])**2))**0.5
        if (excludeGoal):
            return dist <= self.keepConeDistance and not (cone[0] == self.goal[0] and cone[1] == self.goal[1])
        else:
            return dist <= self.keepConeDistance

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

    def scanForCones(self):
        image = self.pilot.robot.get_rgb_image()
        depth = self.pilot.robot.get_depth_image()
        self.cones = img.process(image, depth, self.kRGB)
        print("Found cones: ", self.cones)

    def checkRedCone(self):
        for cone in self.cones:
            if cone.standing and cone.color == "Red":return True
        return False

    def searchRedCone(self, level):
        # first try to rotate
        if level == 4: # base case
            return False
        turned = 0
        while True:
            curAngle = self.pilot.getCurrentPos()[2]
            dA = 1 #rotation step
            turned += dA
            self.pilot.setBearing(curAngle + dA)
            self.scanForCones()
            if(self.checkRedCone()): return True
            if(turned >= 2*3.14 and len(self.cones) > 0): break #full turn
            if (turned > 4*3.14): return False # unable to find a cone
        # rotating did not help, try to drive to a cone but do not touch it
        # firstly choose a cone
        self.cones.sort(reverse=True)
        goal_idx = 0
        for i in range(len(self.cones)):
            self.goal = (self.cones[goal_idx].x, self.cones[goal_idx].y)
            self.findPath(self.goal)
            if (len(self.path > self.acceptedPathLenght + 2)):
                self.path = self.path[:self.acceptedPathLenght]
                break
            elif (len(self.path > 3)):
                self.path = self.path[:-2]
                break
            else:
                goal_idx += 1
                continue
        if(goal_idx == len(self.cones)): # still no path
            goal = [1, 0]
            valid = False
            while not valid:
                valid = True
                for cone in self.cones:
                    if (img.distance(goal, cone) < 2*self.keepConeDistance):
                        valid = False
                        break
                if valid: break
                if (goal[0] > goal[1]):
                    goal[1] += 1
                else: goal[0] += 1
            self.goal = (goal[0], goal[1])
            self.findPath(self.goal)
            self.path = self.path[:5]
        self.pilot.drive(self.path, False)
        return self.searchRedCone(level+1)