from roboticstoolbox.mobile import DubinsPlanner


class DubinsDistanceDict:
    def __init__(self, curvature, stepsize):
        self.curvature = curvature
        self.stepsize = stepsize
        self.DubinsPlanner = DubinsPlanner(curvature=curvature, stepsize=stepsize)
        self.L_d = {}

    def __init__(self, curvature, stepsize, locations, headings):
        self.L_d = {}
        self.curvature = curvature
        self.stepsize = stepsize
        self.DubinsPlanner = DubinsPlanner(curvature=curvature, stepsize=stepsize)
        self.fill(locations, headings)


    def fill(self, locations, headings):
        for location1 in locations:
            self.L_d[location1.idy] = {}
            for heading1 in headings:
                self.L_d[location1.idy][heading1.idy] = {}
                for location2 in locations:
                    self.L_d[location1.idy][heading1.idy][location2.idy] = {}
                    for heading2 in headings:
                        start = [location1.x, location1.y, heading1.angle]
                        goal = [location2.x, location2.y, heading2.angle]
                        query_result = self.DubinsPlanner.query(start=start, goal=goal)
                        distance = query_result[1].length
                        self.L_d[location1.idy][heading1.idy][location2.idy][heading2.idy] = distance

    def get_distance(self, location1, heading1, location2, heading2):
        return self.L_d[location1.idy][heading1.idy][location2.idy][heading2.idy]
