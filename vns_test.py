import math

import pandas as pd
from VNS import VNS
import matplotlib.pyplot as plt

TestSet = pd.read_csv("TestSet1.CSV", sep=";", header=None)
TestSet.columns = ["x", "y", "r"]

radius = 1.3
T_max = 50

if radius == 0:
    curvature = 10.0
else:
    curvature = 1/radius

stepsize = 0.1
m = 6

vns1 = VNS(T_max=T_max,
           curvature=curvature,
           stepsize=stepsize,
           target_locations_df=TestSet,
           m=m,
           l_max=2,
           start_idy=0,
           end_idy=TestSet.shape[0]-1)

p1 = vns1.variable_neighborhood_search(number_of_iterations=5)
print(f"Path found with reward {p1.reward} and distance {p1.distance}")
plot1 = vns1.plot(p1)


