import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from heading import Heading
from location import Location
from path import Path
from dubinsDistanceDict import DubinsDistanceDict


def generate_heading_samples(m):
    """ Generates list of m equidistant headings on [0,2pi) """
    heading_samples = []
    for mi in range(m):
        index = mi + 1
        angle = index/m*(2*np.pi)
        heading = Heading(idy=index, angle=angle)
        heading_samples.append(heading)
    return heading_samples


def target_locations_from_df(target_locations_df):
    target_locations = []
    for idy in target_locations_df.index:
        x = target_locations_df.loc[idy, "x"]
        y = target_locations_df.loc[idy, "y"]
        r = target_locations_df.loc[idy, "r"]
        target_location = Location(idy=idy,
                                   x=x,
                                   y=y,
                                   r=r)
        target_locations.append(target_location)
    return target_locations


class VNS:
    def __init__(self, T_max, radius, stepsize, target_locations_df, m, l_max=2, start_idy=0, end_idy=0):
        self.T_max = T_max
        self.radius = radius
        self.stepsize = stepsize
        self.target_locations = target_locations_from_df(target_locations_df)
        self.m = m
        self.heading_samples = generate_heading_samples(m)
        self.l_max = l_max
        self.start_idy = start_idy
        self.end_idy = end_idy
        self.distance_dict = DubinsDistanceDict(curvature=self.radius,
                                                stepsize=self.stepsize,
                                                locations=self.target_locations,
                                                headings=self.heading_samples)

    def get_reachable_locations(self):
        """qi ∈ Sr ⇔ (Ld(q1,qi)+ Ld(qi,qn )) ≤ Tmax for any combination of sampled heading angles (θ1,θi,θn ).
        This selects all target locations that are reachable by the Dubins vehicle within the travel budget"""
        reachable_locations = []
        start_location = next((location for location in self.target_locations if location.idy == self.start_idy), None)
        end_location = next((location for location in self.target_locations if location.idy == self.end_idy), None)
        reachable_locations.append(start_location)
        reachable_locations.append(end_location)
        for location in self.target_locations:
            if location.idy in [start_location.idy, end_location.idy]:
                continue
            else:
                reachable = True
                for start_heading in self.heading_samples:
                    for heading in self.heading_samples:
                        for end_heading in self.heading_samples:
                            L_1 = self.distance_dict.get_distance(start_location, start_heading, location, heading)
                            L_2 = self.distance_dict.get_distance(location, heading, end_location, end_heading)
                            L = L_1 + L_2
                            if L > self.T_max:
                                reachable = False
                if reachable:
                    reachable_locations.append(location)
        return reachable_locations

    def create_initial_path(self, reachable_locations):
        """For an initial zero reward Dubins path from q1 to qn,
         we iteratively add a new target location from Sr that minimizes
          additional distance per target reward as long as the length of the whole path is below Tmax ."""
        start_location = next((location for location in self.target_locations if location.idy == self.start_idy), None)
        end_location = next((location for location in self.target_locations if location.idy == self.end_idy), None)
        locations_to_add = [location for location in reachable_locations if location.idy not in [self.start_idy, self.end_idy]]
        path = Path(locations=[start_location, end_location],
                    heading_samples=self.heading_samples,
                    distance_dict=self.distance_dict,
                    T_max=self.T_max)
        path1 = path.add_new_locations(locations_to_add)
        return path1

    def shake(self, path, l):
        path1 = path
        if l == 1:
            path1 = path_move(path)
        elif l == 2:
            path1 = path_exchange(path)
        return path1

    def local_search(path,l):
        path1 = path
        len(path.locations)**2
        if l == 1:
            path1 = one_point_move(path)
        elif l == 2:
            path1 = one_point_exchange(path)
        return path1

    def variable_neighborhood_search(self):
        reachable_locations = self.get_reachable_locations()
        path = self.create_initial_path(reachable_locations)
        # stopping_criterion = False
        # while not stopping_criterion:
        #     l = 1
        #     while l <= l_max:
        #         path1 = shake(path, l)
        #         path2 = localSearch(path1, l)
        #         if path2.length <= T_max & path2.get_reward() >= path.get_reward():
        #             path = path2
        #             l = 1
        #         else:
        #             l = l + 1
        print([location.idy for location in path.locations])
        return path

    # def variable_neighborhood_search2(T_max, radius, stepsize, locations, m, l_max=2, start_index=0, end_index=0):
    #     heading_samples = generate_heading_samples(m)
    #     distance_dict = DubinsDistanceDict(curvature=radius, stepsize=stepsize)
    #     distance_dict.fill(locations, heading_samples)
    #     reachable_locations = get_reachable_locations(T_max=T_max,
    #                                                   locations=locations,
    #                                                   heading_samples=heading_samples,
    #                                                   distance_dict=distance_dict,
    #                                                   start_index=start_index,
    #                                                   end_index=end_index)
    #     path = create_initial_path(reachable_locations, T_max, start_index, end_index)
    #     stopping_criterion = False
    #     while not stopping_criterion:
    #         l = 1
    #         while l <= l_max:
    #             path1 = shake(path, l)
    #             path2 = localSearch(path1, l)
    #             if path2.length <= T_max & path2.get_reward() >= path.get_reward():
    #                 path = path2
    #                 l = 1
    #             else:
    #                 l = l + 1
    #     return path
    #



    #plot_rewards1 = plt.scatter(TestSet.x,TestSet.y, s = TestSet.r*2, c = "b")

