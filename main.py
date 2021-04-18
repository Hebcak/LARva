#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot, Rate
import navigator
import time

print("INFO: Hello, I am ConeKiller 2021! I will take care of all the cones!")
print("INFO: Starting initialization")
time.sleep(5)
robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator(robot)
print("INFO: Initialization is complete!")

foundRed = navigator.scanForCones(0)
while (foundRed):
    navigator.toppleRedCone()
    foundRed = navigator.scanForCones(0)
print("INFO: No more cones to topple! Goodbye")






