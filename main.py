#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot, Rate
import navigator
import time
import sys

print("INFO: Hello, I am ConeKiller 2021! I will take care of all the cones!")

colors = colors = {"Red", "Green", "Blue"}
targetColor = "Red"
if (len(sys.argv) > 1):
    if (sys.argv[1] in colors):
        targetColor = sys.argv[1]
    else:
        print("ERROR: invalid color. Next time please select Red, Green, Blue")
print("INFO: Target color is: " + targetColor)

print("INFO: Starting initialization")
time.sleep(5)
robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator(robot, targetColor)
print("INFO: Initialization is complete!")

foundRed = navigator.scanForCones(0)
while (foundRed):
    navigator.toppleTargetCone()
    foundRed = navigator.scanForCones(0)

print("INFO: No more cones to topple! Goodbye")






