from scipy.sparse.csgraph import shortest_path

class Path:
    def __init__(self, locations, heading_samples):
        self.locations = locations
        self.reward = self.get_reward()
        self.heading_samples = heading_samples
        self.optimal_headings = self.calculate_optimal_headings()
        self.distance = self.get_distance()

    def add_new_locations(self,locations_to_add):
        """Adds locations iteratively"""
        current_reward_per_distance_added = 0
        current_path = self
        for location_to_add in locations_to_add:
            reward_to_add = location_to_add.r
            for index in range(len(self.locations)):
                # Create new path path1 with location_to_add at given index
                locations1 = self.locations.copy()
                locations1.insert(location_to_add, index=index)
                path1 = Path(locations = locations1, heading_samples=self.heading_samples)
                reward_per_distance_added1 = (path1.get_reward() - self.get_reward())/(path1.get_distance() - self.get_distance())
                if reward_per_distance_added1 > current_reward_per_distance_added:
                    current_reward_per_distance_added = reward_per_distance_added1
                    current_path = path1


    def get_reward(self):
        R = [location.r for location in self.locations]
        return sum(R)

    def get_headings(self):


    def get_distance(self):
        L = 0
        for i in len(self.locations):

    def calculate_optimal_headings(self):
        pass




