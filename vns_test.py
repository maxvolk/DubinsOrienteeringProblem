import pandas as pd
from VNS import VNS

TestSet = pd.read_csv("TestSet1.CSV", sep=";", header=None)
TestSet.columns = ["x", "y", "r"]

T_max = 50
radius = 1
stepsize = 0.1
m = 4

vns1 = VNS(T_max=T_max,
           radius=radius,
           stepsize=stepsize,
           target_locations_df=TestSet,
           m=m,
           l_max=2,
           start_idy=0,
           end_idy=5)

p1 = vns1.variable_neighborhood_search()
print(p1.get_distance())
