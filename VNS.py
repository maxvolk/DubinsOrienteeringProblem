import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import Heading
import Path


def generate_headings(m):
    """ Generates list of m equidistant headings on [0,2pi] """
    headings = []
    for mi in range(m):
        index = mi + 1
        angle = index/m*(2*np.pi)
        heading = Heading(index=index, angle=angle)
        headings.append(heading)
    return headings


def get_reachable_locations(T_max, locations, headings, distance_dict, start_index, end_index):
    """qi ∈ Sr ⇔ (Ld(q1,qi)+ Ld(qi,qn )) ≤ Tmax for any combination of sampled heading angles (θ1,θi,θn ).
    This selects all target locations that are reachable by the Dubins vehicle within the travel budget"""
    reachable_locations = []
    start_location = next((location for location in locations if location.index == start_index), None)
    reachable_locations.append(start_location)
    end_location = next((location for location in locations if location.index == end_index), None)
    reachable_locations.append(end_location)
    for location in locations:
        if location.index in [start_location.index, end_location.index]:
            continue
        else:
            reachable = True
            for start_heading in headings:
                for heading in headings:
                    for end_heading in headings:
                        L_1 = distance_dict.get_distance(start_location, start_heading, location, heading)
                        L_2 = distance_dict.get_distance(location, heading, end_location, end_heading)
                        L = L_1 + L_2
                        if L > T_max:
                            reachable = False
            if reachable:
                reachable_locations.append(location)
    return reachable_locations


def create_initial_path(T_max, locations, headings, distance_dict, start_index, end_index):
    """For an initial zero reward Dubins path from q1 to qn,
     we iteratively add a new target location from Sr that minimizes
      additional distance per target reward as long as the length of the whole path is below Tmax ."""
    path = Path()

    return path


def variable_neighborhood_search(T_max, radius, stepsize, locations, m, l_max=2, start_index=0, end_index=0):
    headings = generate_headings(m)
    distance_dict = DubinsDistanceDict(curvature=radius, stepsize=stepsize)
    reachable_locations = get_reachable_locations(T_max=T_max,
                                                  locations=locations,
                                                  headings=headings,
                                                  distance_dict=distance_dict,
                                                  start_index=start_index,
                                                  end_index=end_index)
    path = create_initial_path(reachable_locations, T_max, start_index, end_index)
    stopping_criterion = False
    while not stopping_criterion:
        l = 1
        while l <= l_max:
            path1 = shake(path, l)
            path2 = localSearch(path1, l)
            if path2.length <= T_max & path2.get_reward() >= path.get_reward():
                path = path2
                l = 1
            else:
                l = l + 1
    return path


TestSet = pd.read_csv("TestSet1.CSV",sep = ";", header=None)
TestSet.columns = ["x","y","r"]
TestSet

#plot_rewards1 = plt.scatter(TestSet.x,TestSet.y, s = TestSet.r*2, c = "b")

