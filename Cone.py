
class Cone:
    '''Class to store found cones'''
    def __init__(self, color, coord, standing):
        self.color = color
        self.coord = coord
        self.standing = standing
    def __repr__(self):      # for printing
        stand = "standing" if self.standing else "fallen"
        return "\n{position: " + str(self.coord) + ", color: '" + self.color + "', state: " + stand + "}"