import numpy as np
class Coordinates:
    '''Class to store coordinates'''
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):  # for printing
        return "[%.3f, %.3f, %.3f]" % (self.x, self.y, self.z)
    def numpyfy(self):
        return np.array([self.x, self.y, self.z])