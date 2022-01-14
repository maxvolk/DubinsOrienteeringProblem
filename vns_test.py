import math

import pandas as pd
from VNS import VNS
import matplotlib.pyplot as plt

TestSet = pd.read_csv("TestSet3.CSV", sep=";", header=None)
TestSet.columns = ["x", "y", "r"]

radius = 1
T_max = 40

if radius == 0:
    curvature = 1000.0
else:
    curvature = 1/radius

stepsize = 0.1
m = 8

T_initial = T_max

vns1 = VNS(T_max=T_max,
           T_initial=T_initial,
           curvature=curvature,
           stepsize=stepsize,
           target_locations_df=TestSet,
           m=m,
           l_max=2,
           start_idy=0,
           end_idy=TestSet.shape[0]-1)

p1 = vns1.variable_neighborhood_search(number_of_iterations=1000,
                                       consider_all_angles=False,
                                       only_initial_solution=False)
print(f"Path found with reward {p1.reward} and distance {p1.distance}")
plot1 = vns1.plot(p1)


