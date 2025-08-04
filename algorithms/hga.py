
"""
    Reference:  Ha, Quang Minh, et al. 
                "A hybrid genetic algorithm for the traveling salesman problem with drone." 
                Journal of Heuristics 26 (2020).
"""


from config import config

### CONFIG:
drone_velocity = config['drone_velocity']
drone_battery = config['drone_battery']
drone_launch_time = config['drone_launch_time']
drone_recover_time = config['drone_recover_time']
drone_service_time = config['drone_service_time']
drone_capacity = config['drone_capacity']

truck_velocity = config['truck_velocity']
truck_service_time = config['truck_service_time']

maxIteration = 20000

# GA hyperparameters
max_num_pop = 30

## END CONFIG


class Solution:
    def __init__(self, truck_route, truck_time, drone_routes, delta_D):
        self.truck_route = truck_route
        self.drone_routes = drone_routes
        self.truck_time = truck_time

        self.fitness = self.get_time_span_objective(delta_D)

    def get_time_span_objective(self, delta_D):

        """ Output the time to complete mission. """

        for drone_arc in self.drone_routes:
            if drone_arc[2] == 0:   # drone going back to depot itself
                launching_time = self.truck_time[self.truck_route.index(drone_arc[0])]

                return_time = (launching_time +
                               drone_launch_time + 
                               delta_D[drone_arc[0], drone_arc[1]] +
                               drone_service_time +
                               delta_D[drone_arc[1], drone_arc[2]] +
                               drone_recover_time)
                
                if self.truck_time[-1] < return_time:   # drone go back to depot after the truck
                    return return_time
                
        return self.truck_time[-1]
    


def hgs(numnodes, parcel_weight, delta_T, delta_D):

    """ Input:
        
            numnodes:       number of nodes, including depot
            parcel_weight:  the weight of package need to deliver to customer at each node
            delta_T:        adjaciency traveling matrix (in time, sec) of the truck
            delta_D:        adjaciency traveling matrix (in time, sec) of the drone.
            
        Output:

            

        """
    
    # Init population
    pop =  generate_random_pop(numnodes, max_num_pop)

    best_sol = []

    for iter in range(maxIteration):
        # Select parent P1 and P2
        P1, P2 = select_parents(pop)

        # Generate offspring C from parents P1 and P2
        C = []
        # Apply split on C

        # Educate C using local search

        # Call restore method to update the giant-tour chromosome in C

        if not is_feasible(C):
            # insert C into infeasible subpopulation

            # repair C with probability P_rep
            pass
        if is_feasible(C):
            # insert C into feasible subpopulation
            pass

        if len(pop) > max_num_pop: 
            # select survivors
            pass

        # adjust the penalty parameters for violating the drone endurance constraints

        # if best_sol is not improved for iter_div iterations: -> diversify pop

    return best_sol
        
def generate_random_pop(numnodes, num_sol):
    solutions = []

    for i in range(num_sol):
        # TODO:
        # generate a randome solution
        sol = []

        # add it to the set
        solutions.append(sol)

    return solutions

def select_parents(pop):
    # TODO:
    p1 = pop[0]
    p2 = pop[1]
    return p1, p2

def is_feasible(sol):
    # TODO:
    return True