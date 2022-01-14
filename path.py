import networkx as nx

class Path:
    def __init__(self, permutation, locations, heading_samples, distance_dict, T_max):  # search_graph,
        self.permutation = permutation
        self.locations = locations
        self.reward = self.get_reward()
        self.heading_samples = heading_samples
        self.distance_dict = distance_dict
        #self.search_graph = search_graph
        self.optimal_headings = self.calculate_optimal_headings()
        self.distance = self.get_distance()
        self.T_max = T_max
        self.is_feasible = self.distance <= self.T_max


    def add_new_locations(self, locations_to_add):
        """Adds locations iteratively in order of decreasing added reward per added distance"""
        locations_to_add = locations_to_add.copy()
        current_reward_per_distance_added = 0
        current_best_path = self
        best_location_to_add = None
        for location_to_add in locations_to_add:
            for index in range(1, len(self.locations)):
                # Create new path path1 with location_to_add at given index
                locations1 = self.locations.copy()
                locations1.insert(index, location_to_add)
                permutation1 = self.permutation.copy()
                permutation1.remove(location_to_add.idy)
                permutation1.insert(index, location_to_add.idy)
                path1 = Path(permutation=permutation1,
                             locations=locations1,
                             heading_samples=self.heading_samples,
                             distance_dict=self.distance_dict,
                             T_max=self.T_max)
                reward_per_distance_added1 = (path1.reward - self.reward)/(path1.distance - self.distance)
                if (reward_per_distance_added1 > current_reward_per_distance_added) & path1.is_feasible:
                    current_reward_per_distance_added = reward_per_distance_added1
                    current_best_path = path1
                    best_location_to_add = location_to_add
                    continue
        if best_location_to_add is not None:
            locations_to_add.remove(best_location_to_add)
            print(str(best_location_to_add.idy) + " added")
            # print("new locations_to_add" + str([la.idy for la in locations_to_add]))
            if len(locations_to_add) >= 1:
                current_best_path = current_best_path.add_new_locations(locations_to_add)
        return current_best_path

    def get_reward(self):
        R = [location.r for location in self.locations]
        return sum(R)

    def get_optimal_headings(self):
        return self.optimal_headings

    def get_distance(self):
        distance = 0
        for i in range(1, len(self.locations)):
            distance = distance + self.distance_dict.get_distance(location1=self.locations[i-1],
                                                                  heading1=self.optimal_headings[i-1],
                                                                  location2=self.locations[i],
                                                                  heading2=self.optimal_headings[i])
        return distance

    def calculate_optimal_headings(self):
        G = self.generate_search_graph()
        # G = self.search_graph
        shortest_path_in_g = nx.shortest_path(G,
                                              source="start",
                                              target="end",
                                              weight="weight",
                                              method="dijkstra")
        optimal_headings = []
        for node in shortest_path_in_g[1:len(shortest_path_in_g)-1]:
            heading_idy = node[1]
            heading = next((heading for heading in self.heading_samples if heading.idy == heading_idy), None)
            optimal_headings.append(heading)
        self.optimal_headings = optimal_headings
        return optimal_headings

    def generate_search_graph(self):
        G = nx.Graph()
        for location in self.locations:
            for heading in self.heading_samples:
                G.add_node((location.idy, heading.idy))

        for i in range(1, len(self.locations)):
            for heading1 in self.heading_samples:
                for heading2 in self.heading_samples:
                    location1 = self.locations[i-1]
                    location2 = self.locations[i]
                    G.add_edge(u_of_edge=(location1.idy, heading1.idy),
                               v_of_edge=(location2.idy, heading2.idy),
                               weight=self.distance_dict.get_distance(location1=location1,
                                                                      heading1=heading1,
                                                                      location2=location2,
                                                                      heading2=heading2))

        G.add_node("start")
        G.add_node("end")
        for heading in self.heading_samples:
            G.add_edge(u_of_edge="start",
                       v_of_edge=(self.locations[0].idy, heading.idy),
                       weight=0)
            G.add_edge(u_of_edge=(self.locations[len(self.locations)-1].idy, heading.idy),
                       v_of_edge="end",
                       weight=0)
        return G








