import random

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from dubins_path_planning import dubins_path_planning
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


def generate_cutpoints(list_to_cut, number_of_cutpoints):
    """Generates distinct and sorted cutpoints on list that are random but not allowed to be the first index"""
    while True:
        r = []
        for i in range(number_of_cutpoints):
            r.append(random.randint(1, len(list_to_cut)-1))  # only the index 0 is not allowed
        if len(set(r)) == number_of_cutpoints:
            break
    r.sort()
    return r


class VNS:
    def __init__(self, T_max, T_initial, curvature, stepsize, target_locations_df, m, l_max=2, start_idy=0, end_idy=0):
        self.T_max = T_max
        self.T_initial = T_initial
        self.curvature = curvature
        self.stepsize = stepsize
        self.target_locations = target_locations_from_df(target_locations_df)
        self.target_locations_idys = [location.idy for location in self.target_locations]
        self.m = m
        self.heading_samples = generate_heading_samples(m)
        self.l_max = l_max
        self.start_idy = start_idy
        self.end_idy = end_idy
        print("Filling distance dict")
        self.distance_dict = DubinsDistanceDict(curvature=self.curvature,
                                                stepsize=self.stepsize,
                                                locations=self.target_locations,
                                                headings=self.heading_samples)

    def variable_neighborhood_search(self, number_of_iterations, consider_all_angles, only_initial_solution):
        print("Calculating reachable locations")
        reachable_locations = self.get_reachable_locations(consider_all_angles=consider_all_angles)
        print("Creating initial path")
        path = self.create_initial_path(reachable_locations)
        print(f"Initial path with reward {path.reward} and distance {path.distance} built")
        if not only_initial_solution:
            iteration = 1
            while not iteration >= number_of_iterations:
                l = 1
                while l <= self.l_max:
                    path1 = self.shake(path, l)
                    path2 = self.local_search(path1, l)
                    if path2.distance <= self.T_max & path2.reward > path.reward:
                        path = path2
                        l = 1
                    else:
                        if path2.distance > self.T_max:
                            print("Path2 > T")
                        if path2.reward < path.reward:
                            print("Path2 < R")
                        l = l + 1
                iteration = iteration + 1
                print(f"Iteration {iteration}, Current best path (Reward: {path.reward}, Distance: {path.distance})")
        print([location.idy for location in path.locations])
        return path

    def get_reachable_locations(self, consider_all_angles=True):
        """qi ∈ Sr ⇔ (Ld(q1,qi)+ Ld(qi,qn )) ≤ Tmax for any combination of sampled heading angles (θ1,θi,θn ).
        This selects all target locations that are reachable by the Dubins vehicle within the travel budget"""
        reachable_locations = [] #self.target_locations.copy()
        start_location = next((location for location in self.target_locations if location.idy == self.start_idy), None)
        end_location = next((location for location in self.target_locations if location.idy == self.end_idy), None)
        reachable_locations.append(start_location)
        reachable_locations.append(end_location)
        for location in self.target_locations:
            if location.idy in [start_location.idy, end_location.idy]:
                continue
            else:
                if consider_all_angles:
                    reachable = True
                else:
                    reachable = False
                for start_heading in self.heading_samples:
                    for heading in self.heading_samples:
                        for end_heading in self.heading_samples:
                            L_1 = self.distance_dict.get_distance(start_location, start_heading, location, heading)
                            L_2 = self.distance_dict.get_distance(location, heading, end_location, end_heading)
                            L = L_1 + L_2
                            if (L > self.T_max) & consider_all_angles:
                                reachable = False
                            elif (L <= self.T_max) & (not consider_all_angles):
                                reachable = True
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
        initial_permutation = [self.start_idy, self.end_idy] + [idy for idy in self.target_locations_idys if idy not in [self.start_idy, self.end_idy]]
        path = Path(permutation=initial_permutation,
                    locations=[start_location, end_location],
                    heading_samples=self.heading_samples,
                    distance_dict=self.distance_dict,
                    T_max=self.T_initial)
        path1 = path.add_new_locations(locations_to_add)
        return path1

    def shake(self, path, l):
        path1 = path
        if l == 1:
            path1 = self.path_move(path)
        elif l == 2:
            path1 = self.path_exchange(path)
        return path1

    def path_move(self, path):
        r = generate_cutpoints(path.permutation, 2)
        location_idys_to_be_moved = path.permutation[r[0]:r[1]+1]
        location_idys_to_be_kept = [path.permutation[idx] for idx in range(0, len(path.permutation)) if idx not in range(r[0], r[1]+1)]
        if len(location_idys_to_be_kept) <= 2:
            o = [0]
        else:
            o = generate_cutpoints(location_idys_to_be_kept, 1)
        for idx in range(0, len(location_idys_to_be_moved)):
            location_idys_to_be_kept.insert(o[0]+idx+1, location_idys_to_be_moved[idx])
        permutation1 = location_idys_to_be_kept
        locations1 = self.get_locations_from_permutation(permutation=permutation1)
        print(permutation1)
        path1 = Path(permutation=permutation1,
                     locations=locations1,
                     heading_samples=self.heading_samples,
                     distance_dict=self.distance_dict,
                     T_max=self.T_max)
        return path1

    def path_exchange(self, path):
        r = generate_cutpoints(path.permutation, 4)
        location_idys_to_be_kept1 = path.permutation[0:r[0]]
        location_idys_to_be_moved1 = path.permutation[r[0]:r[1]+1]
        location_idys_to_be_kept2 = path.permutation[r[1]+1:r[2]]
        location_idys_to_be_moved2 = path.permutation[r[2]:r[3]+1]
        location_idys_to_be_kept3 = path.permutation[r[3]+1:len(path.permutation)]
        permutation1 = location_idys_to_be_kept1 + location_idys_to_be_moved2 + location_idys_to_be_kept2 + location_idys_to_be_moved1 + location_idys_to_be_kept3
        locations1 = self.get_locations_from_permutation(permutation=permutation1)
        path1 = Path(permutation=permutation1,
                     locations=locations1,
                     heading_samples=self.heading_samples,
                     distance_dict=self.distance_dict,
                     T_max=self.T_max)
        return path1

    def local_search(self, path, l):
        number_of_iterations = len(path.locations)**2
        current_best_path = path
        if l == 1:
            for i in range(number_of_iterations):
                path1 = self.one_point_move(path)
                if path1.reward > current_best_path.reward:
                    current_best_path = path1
        elif l == 2:
            for i in range(number_of_iterations):
                path1 = self.one_point_exchange(path)
                if path1.reward > current_best_path.reward:
                    current_best_path = path1
        return current_best_path

    def one_point_move(self, path):
        r = generate_cutpoints(path.permutation, 1)
        permutation1 = [path.permutation[idx] for idx in range(0, len(path.permutation)) if idx not in r]
        r_insert = generate_cutpoints(permutation1, 1)
        permutation1.insert(r_insert[0], path.permutation[r[0]])
        locations1 = self.get_locations_from_permutation(permutation=permutation1)
        path1 = Path(permutation=permutation1,
                     locations=locations1,
                     heading_samples=self.heading_samples,
                     distance_dict=self.distance_dict,
                     T_max=self.T_max)
        return path1

    def one_point_exchange(self, path):
        r = generate_cutpoints(path.permutation, 2)
        location_idys_to_be_kept1 = path.permutation[0:r[0]]
        location_idys_to_be_moved1 = path.permutation[r[0]:r[0]+1]
        location_idys_to_be_kept2 = path.permutation[r[0]+1:r[1]]
        location_idys_to_be_moved2 = path.permutation[r[1]:r[1]+1]
        location_idys_to_be_kept3 = path.permutation[r[1]+1:len(path.permutation)]
        permutation1 = location_idys_to_be_kept1 + location_idys_to_be_moved2 + location_idys_to_be_kept2 + location_idys_to_be_moved1 + location_idys_to_be_kept3
        locations1 = self.get_locations_from_permutation(permutation=permutation1)
        path1 = Path(permutation=permutation1,
                     locations=locations1,
                     heading_samples=self.heading_samples,
                     distance_dict=self.distance_dict,
                     T_max=self.T_max)
        return path1

    def get_locations_from_permutation(self, permutation):
        location_idys_path = permutation[permutation.index(self.start_idy):permutation.index(self.end_idy)+1]
        locations = []
        for location_idy in location_idys_path:
            locations.append(next((location for location in self.target_locations if location.idy == location_idy), None))
        return locations

    def plot(self, path):
        path_x = []
        path_y = []
        for idx in range(len(path.locations)-1):
            location1 = path.locations[idx]
            heading1 = path.optimal_headings[idx]
            location2 = path.locations[idx+1]
            heading2 = path.optimal_headings[idx+1]
            # start = [location1.x, location1.y, heading1.angle]
            # goal = [location2.x, location2.y, heading2.angle]
            # query_result = self.distance_dict.DubinsPlanner.query(start=start, goal=goal)
            # path_array_i = [[a[0], a[1]] for a in query_result[0]]
            # path_array = path_array + path_array_i
            path_x_i, path_y_i, path_yaw_i, mode_i, lengths_i = dubins_path_planning(s_x=location1.x,
                                                                                     s_y=location1.y,
                                                                                     s_yaw=heading1.angle,
                                                                                     g_x=location2.x,
                                                                                     g_y=location2.y,
                                                                                     g_yaw=heading2.angle,
                                                                                     curvature=self.curvature,
                                                                                     step_size=self.stepsize)
            length_i = sum(lengths_i)
            print(f"Length from {location1.idy} to {location2.idy}: {length_i}")
            path_x = np.concatenate((path_x, path_x_i), axis=None)
            path_y = np.concatenate((path_y, path_y_i), axis=None)
        # x_path = [a[0] for a in path_array]
        # y_path = [a[1] for a in path_array]
        fig, ax = plt.subplots()
        x_targets = [location.x for location in self.target_locations]
        y_targets = [location.y for location in self.target_locations]
        r_targets = [location.r for location in self.target_locations]
        sc = ax.scatter(x_targets, y_targets, label="Target locations", c=r_targets)
        ax.plot(path_x, path_y, label="Dubins path")
        ax.legend()
        ax.set_aspect("equal")
        cbar = plt.colorbar(sc, label="Reward")
        fig.show()
        return fig, ax

    # plot_rewards1 = plt.scatter(TestSet.x,TestSet.y, s = TestSet.r*2, c = "b")

