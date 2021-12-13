# from roboticstoolbox.mobile import DubinsPlanner
# from DubinsPlanner import DubinsPlanner
from dubins_path_planning import *


class DubinsDistanceDict:
    def __init__(self, curvature, stepsize):
        self.curvature = curvature
        self.stepsize = stepsize
        # self.DubinsPlanner = DubinsPlanner(curvature=curvature, stepsize=stepsize)
        self.L_d = {}

    def __init__(self, curvature, stepsize, locations, headings):
        self.L_d = {}
        self.curvature = curvature
        self.stepsize = stepsize
        # self.DubinsPlanner = DubinsPlanner(curvature=curvature, stepsize=stepsize)
        self.fill(locations, headings)

    def fill(self, locations, headings):
        for location1 in locations:
            self.L_d[location1.idy] = {}
            for heading1 in headings:
                self.L_d[location1.idy][heading1.idy] = {}
                for location2 in locations:
                    self.L_d[location1.idy][heading1.idy][location2.idy] = {}
                    for heading2 in headings:
                        #start = [location1.x, location1.y, heading1.angle]
                        #goal = [location2.x, location2.y, heading2.angle]
                        #query_result = self.DubinsPlanner.query_fast(start=start, goal=goal)
                        #distance = query_result[1].length
                        path_x, path_y, path_yaw, mode, lengths = dubins_path_planning_fast(s_x=location1.x,
                                                                                       s_y=location1.y,
                                                                                       s_yaw=heading1.angle,
                                                                                       g_x=location2.x,
                                                                                       g_y=location2.y,
                                                                                       g_yaw=heading2.angle,
                                                                                       curvature=self.curvature,
                                                                                       step_size=self.stepsize)
                        distance = sum(lengths)
                        self.L_d[location1.idy][heading1.idy][location2.idy][heading2.idy] = distance

    def get_distance(self, location1, heading1, location2, heading2):
        return self.L_d[location1.idy][heading1.idy][location2.idy][heading2.idy]
