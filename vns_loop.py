import time
import pandas as pd
from VNS import VNS
import matplotlib.pyplot as plt
i = 0
number_of_iterations = 1000
stepsize = 0.1
results = pd.DataFrame(columns=["TestSetFile",
                                "radius",
                                "T_max",
                                "T_initial",
                                "m",
                                "number_of_iterations",
                                "computational_time",
                                "reward",
                                "distance"])
consider_all_angles = False

for TestSetFile in ["TestSet1.CSV"]: #, "TestSet2.CSV", "TestSet3.CSV"]:
    for radius in [0]: #, 1]:
        for T_max in [20, 40, 60]:
            for m in [1]: #[4, 8]:
                start_time = time.time()
                TestSet = pd.read_csv(TestSetFile, sep=";", header=None)
                TestSet.columns = ["x", "y", "r"]
                if radius == 0:
                    curvature = 1000.0
                else:
                    curvature = 1/radius

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

                p1 = vns1.variable_neighborhood_search(number_of_iterations=number_of_iterations,
                                                       consider_all_angles=consider_all_angles,
                                                       only_initial_solution=True)
                print(f"Path found with reward {p1.reward} and distance {p1.distance}")
                # plot1 = vns1.plot(p1)
                end_time = time.time()
                computational_time = end_time - start_time
                results.loc[i] = [TestSetFile,
                                  radius,
                                  T_max,
                                  T_initial,
                                  m,
                                  number_of_iterations,
                                  computational_time,
                                  p1.reward,
                                  p1.distance]
                i = i + 1

results.to_csv("results1_initial_notAllAngles_zeros.CSV")