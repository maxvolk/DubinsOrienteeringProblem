from roboticstoolbox.mobile import DubinsPlanner


class DubinsDistanceDict:
    def __init__(self, curvature, stepsize):
        self.curvature = curvature
        self.stepsize = stepsize
        self.DubinsPlanner = DubinsPlanner(curvature=curvature, stepsize=stepsize)
        self.L_d = {}

    def fill(self, locations, headings):
        for location1 in locations:
            for heading1 in headings:
                for location2 in locations:
                    for heading2 in headings:
                        start = [location1.x, location1.y, heading1.angle]
                        goal = [location2.x, location2.y, heading2.angle]
                        distance = self.DubinsPlanner.query(start=start, goal=goal)
                        self.L_d[location1.index][heading1.index][location2.index][heading2.index] = distance

    def get_distance(self, location1, heading1, location2, heading2):
        return self.L_d[location1.index][heading1.index][location2.index][heading2.index]
