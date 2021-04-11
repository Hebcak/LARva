
class Cone:
    '''Class to store found cones'''
    def __init__(self, color, coord, standing):
        self.color = color
        self.coord = coord
        self.standing = standing

    def __repr__(self):      # for printing
        stand = "standing" if self.standing else "fallen"
        # Python 3
        # return "position: [{:.3}, {:.3}, {:.3}]".format(self.x, self.y, self.z) + ", color: '" + self.color + "', state: " + stand
        # Python 2
        return "\n{position: " + str(self.coord) + ", color: '" + self.color + "', state: " + stand + "}"

    def __lt__(self, other): # for comparing two nodes
        if not self.standing: return True
        if not other.standing: return False
        if (abs(self.coord.y) < abs(other.coord.y)): return True
        return False

    def __gt__(self, other):  # for comparing two nodes
        if not self.standing: return False
        if not other.standing: return True
        if(abs(self.coord.y) > abs(other.coord.y)): return True
        return False