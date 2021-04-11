#The main file -> TO RUN THIS PROJECT run this file
from robolab_turtlebot import Turtlebot, Rate
import navigator

robot = Turtlebot(rgb=True, depth=True)
navigator = navigator.Navigator(robot)
Rate(1100).sleep()

navigator.scanForCones()
while True:
    if navigator.checkRedCone():
        navigator.toppleRedCone()
        Rate(1100).sleep() # wait until cone falls
        navigator.scanForCones()
    else:
        if not navigator.searchRedCone(0):
            break # no red cone, end the program






