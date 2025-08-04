"""
    Reference:  Das, Dyutimoy Nirupam, et al. 
                "Synchronized truck and drone routing in package delivery logistics." 
                IEEE Transactions on Intelligent Transportation Systems 22.9 (2020)
"""

import numpy as np
import random

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

# ACO hyperparameters
num_ant = 10
alpha = 1
beta = 1

C = 1000

tau_0 = 1           # initial pheromone level
tau_max = 20        # max pheromone level
tau_min = 1         # min pheromone level

rho_local = 0.05    # evaporation rate eq.30
rho_global = 0.05   # evaporation rate eq.37

elite_max_size = 30 # maximum solution saved on elite archive

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

def cp_aco(numnodes, parcel_weight, delta_T, delta_D):

    """ Input:
        
            numnodes:       number of nodes, including depot
            parcel_weight:  the weight of package need to deliver to customer at each node
            delta_T:        adjaciency traveling matrix (in time, sec) of the truck
            delta_D:        adjaciency traveling matrix (in time, sec) of the drone.
            
        Output:

            Pareto optimal front

        """

    # Initialize pheromone matrix
    tau_t = np.ones((numnodes, numnodes))  # pheromone matrix for the truck
    tau_d = np.ones((numnodes, numnodes))  # pheromone matrix for the drone

    # Calculate eta matrices based on delta_T, delta_D
    # eta_t = C / delta_T
    # eta_d = C / delta_D

    eta_t = np.zeros((numnodes,numnodes))
    for i in range(numnodes):
        eta_t[i,i] = np.inf
        for j in range(i+1,numnodes):
            eta_t[i,j] = C/delta_T[i,j]
            eta_t[j,i] = eta_t[i,j]

    eta_d = np.zeros((numnodes,numnodes))
    for i in range(numnodes):
        eta_d[i,i] = np.inf
        for j in range(i+1,numnodes):
            eta_d[i,j] = C/delta_D[i,j]
            eta_d[j,i] = eta_d[i,j]

    # Elite Archive
    elite_archive = []

    print('CP-ACO calculating ...')

    # Main loop
    for iter in range(maxIteration):
        for ant in range(num_ant):
            customer = list(range(1, numnodes))   # available customer, except of the depot


            # STAGE 1:Constructing initial TSP solution for the truck
            last_node_i = 0       # starting from the depot

            # truck solution
            truck_route = [last_node_i]
            truck_time = [0]

            while len(customer) > 0:
                # evaluate p_{ij} for last node i 
                p = (tau_t[last_node_i, :] ** alpha) * (eta_t[last_node_i, :] ** beta) 

                # zero out the invalid customer
                p = [p[i] if i in customer else 0 for i in range(len(p))]

                if sum(p) == 0:
                    raise ValueError()
                
                # convert it to probabilities
                p = p / sum(p)

                next_node_j = roulette_wheel_selection(p)

                # update the tour time
                truck_time.append(truck_time[-1] + delta_T[last_node_i, next_node_j])
                if last_node_i != 0:
                    truck_time[-1] += truck_service_time     # adding service time at prev customer node

                # remove node j from customer
                customer.remove(next_node_j)

                # add it to the truck path
                truck_route.append(next_node_j)

                # Local phenomone update
                tau_t[last_node_i, next_node_j] = (1 - rho_local)*tau_t[last_node_i, next_node_j] + rho_local*tau_0

                # update last_node value
                last_node_i = next_node_j

            # add the depot to the end
            truck_route.append(0)
            truck_time.append(truck_time[-1] + delta_T[last_node_i, next_node_j] + truck_service_time)



            # STAGE 2: Drone path creation
            drone_routes = []

            for i in range(len(truck_route)-1):

                if i >= len(truck_route)-1:
                    break

                if not drone_is_on_subtour(i, truck_route, drone_routes):
                    # find possible drone sub tour
                    drone_tours = possible_drone_tour(truck_route[i:], truck_time[i:], parcel_weight, delta_D)

                    # choosing sub tour: calculating selection probs for each subtour
                    num_drone_tours = len(drone_tours)
                    if num_drone_tours == 0:
                        continue

                    p_ijk = []

                    for i_tour in range(num_drone_tours):
                        tour = drone_tours[i_tour]
                        node_i = tour[0]
                        node_j = tour[1]
                        node_k = tour[2]

                        # the change in the cost of travel in truck path due to subtour
                        epsilon_ijk = calculate_epsilon(truck_route, truck_time, tour, delta_T, delta_D)

                        p_value = (((tau_d[node_i,node_j]*tau_d[node_j, node_k]) ** alpha) * 
                                   (eta_d[node_i,node_j]*eta_d[node_j, node_k]) ** beta * 
                                   epsilon_ijk ** beta)
                        
                        p_ijk.append(p_value)
                    
                    # convert to the probs
                    p_ijk = np.array(p_ijk)

                    if sum(p_ijk) == 0:
                        continue

                    p_ijk = p_ijk / sum(p_ijk)

                    # choose the tour
                    c_tour = roulette_wheel_selection(p_ijk)
                    choosed_i = drone_tours[c_tour][0]
                    choosed_j = drone_tours[c_tour][1]
                    choosed_k = drone_tours[c_tour][2]

                    # Update solution
                    drone_routes.append([choosed_i,choosed_j,choosed_k])
                    truck_route.remove(choosed_j)
                    truck_time = recalc_time(truck_route, drone_routes,delta_T,delta_D)

            # Add new solution to ELite Archive
            new_sol = Solution(truck_route, truck_time, drone_routes, delta_D)
            elite_archive = update_elite(elite_archive, new_sol, elite_max_size)

        # Global phenomone update 
        for solution in elite_archive:
            
            objective_value = solution.fitness
            added_value = C / objective_value

            for i in range(len(solution.truck_route)-1):
                node1 = solution.truck_route[i]
                node2 = solution.truck_route[i+1]

                tau_t[node1,node2] += added_value

            for ri in range(len(solution.drone_routes)):
                node_i = solution.drone_routes[ri][0]
                node_j = solution.drone_routes[ri][1]
                node_k = solution.drone_routes[ri][2]

                tau_d[node_i,node_j] += added_value
                tau_d[node_j,node_k] += added_value

        # Evaporation
        tau_t = (1 - rho_global)*tau_t
        tau_d = (1 - rho_global)*tau_d

        # bounded with tau_max and tau_min
        # tau_t = [
        #     [min(max(tau_ij, tau_min), tau_max) for tau_ij in row]
        #     for row in tau_t
        # ]

        # tau_d = [
        #     [min(max(tau_ij, tau_min), tau_max) for tau_ij in row]
        #     for row in tau_d
        # ]

        # Print at each iteration
        # print(f'Iteration {iter}: Length of Elite Archive: {len(elite_archive)} \t Sampled solution: {random.choice(elite_archive).fitness}')

    best_solution = min(elite_archive, key=lambda sol: sol.fitness)

    returned_truck_route = best_solution.truck_route
    returned_truck_time = best_solution.truck_time
    returned_drone_route = best_solution.drone_routes
    return_timespan = best_solution.fitness


    return returned_truck_route, returned_truck_time, returned_drone_route, return_timespan
                

def possible_drone_tour(truck_sub_tour, truck_time, parcel_weight, delta_D):
    """
        Input: truck_sub_tour   the truck tour from i to end (i is the current node)
               truck_time        the time that truck comes to each node in the above array
               parcel_weight    specifies which task is feasible for drone delivery
        
        Ouput: all possible drone tour starting at i 
    """

    node_i = truck_sub_tour[0]      # launching node

    drone_tours = []

    for j_index in range(1, len(truck_sub_tour)):
        
        node_j = truck_sub_tour[j_index]    # customer node

        # infeasible node
        if parcel_weight[node_j] > drone_capacity:
            break

        if delta_D[node_i, node_j] > drone_battery:
            break

        for k_index in range(j_index+1, len(truck_sub_tour)-1):
            
            node_k = truck_sub_tour[k_index]    # landing node

            drone_travel_time = (drone_launch_time + 
                                 delta_D[node_i,node_j] + 
                                 drone_service_time +
                                 delta_D[node_j, node_k] + 
                                 drone_recover_time)
            
            truck_travel_time = truck_time[k_index] - truck_time[0]

            travel_time = max(drone_travel_time, truck_travel_time)

            if travel_time < drone_battery:

                # adding subtour to the drone_feasible_tours
                drone_tours.append([node_i, node_j, node_k])

    return drone_tours

def drone_is_on_subtour(node_idx, truck_route, drone_routes):

    """ Return True if at this current truck node, drone is flying, otherwise False. """

    for route in drone_routes:
        launch_node = route[0]
        land_node = route[2]

        if truck_route.index(launch_node) < node_idx and truck_route.index(land_node) > node_idx:
            return True
        
    return False

def roulette_wheel_selection(P):
    r = random.random()
    C = [sum(P[:i + 1]) for i in range(len(P))]

    for j, c in enumerate(C):
        if r <= c:
            return j
        
def calculate_epsilon(truck_route, truck_time, drone_arc, delta_T, delta_D):
    """
        Input: 
                truck_route     The tsp route assgined for the truck
                truck_time       The time truck comes to the nodes in its route
                drone_arc       The (i,j,k) sequence at which drone plan to form a sub tour
    
        Output: the advantage value (saved time) when assigning the drone to subtour, instead of truck
    """

    node_i = drone_arc[0]   # launching node
    node_j = drone_arc[1]   # customer node
    node_k = drone_arc[2]   # retriving node

    j_in_truck = truck_route.index(node_j)
    pred_j = truck_route[j_in_truck-1]
    succ_j = truck_route[j_in_truck+1]

    saving_truck = delta_T[pred_j,node_j] + delta_T[node_j, succ_j] - delta_T[pred_j, succ_j]

    truck_at_k = truck_time[truck_route.index(node_k)] - saving_truck

    drone_at_k = (truck_time[truck_route.index(node_i)] +
                  drone_launch_time + delta_D[node_i,node_j] + 
                  drone_service_time + delta_D[node_j,node_k] + 
                  drone_recover_time)
    
    epsilon = truck_time[truck_route.index(node_k)] - max(truck_at_k, drone_at_k)

    return epsilon


def recalc_time(truck_route, drone_routes, delta_T, delta_D):

    """Output: a new time_to_node value"""

    # output
    time_to_node = [0]

    # get list of drone launch node and drone_retrieve node
    drone_launch = []
    drone_custom = []
    drone_retrie = []

    for route in drone_routes:
        # launch node -> customer node -> retrieve node
        drone_launch.append(route[0])
        drone_custom.append(route[1])
        drone_retrie.append(route[2])

    # calulating time
    for j in range(1, len(truck_route)):
        node_j = truck_route[j]
        prev_node = truck_route[j-1]

        travel_time = delta_T[prev_node,node_j]

        if prev_node != 0:  # add service time of previous node
            travel_time += truck_service_time

        arrival_time = time_to_node[-1] + travel_time
        
        drone_arrival_time = 0
        if node_j in drone_retrie:
            node_a = drone_launch[drone_retrie.index(node_j)]      
            node_b = drone_custom[drone_retrie.index(node_j)]

            drone_arrival_time = (
                time_to_node[truck_route.index(node_a)] +
                delta_D[node_a, node_b] +
                delta_D[node_b, node_j] +
                drone_launch_time +
                drone_service_time +
                drone_recover_time
            )

        arrival_time = max(arrival_time, drone_arrival_time)
        time_to_node.append(round(float(arrival_time), 2))

    return time_to_node


def update_elite(elite_archive, new_sol, max_size):

    """ Append new solution and remove crowded solution. """

    elite_archive.append(new_sol)

    while len(elite_archive) > max_size:

        to_removed = -1
        min_dist = float('inf')

        for i in range(len(elite_archive)):
            solution = elite_archive[i]

            # Calculate total distance to other solutions
            dist = sum(abs(solution.fitness - other.fitness) 
                       for j, other in enumerate(elite_archive) if j != i)

            if min_dist < dist:
                to_removed = i
                min_dist = dist

        del elite_archive[to_removed]

    return elite_archive
