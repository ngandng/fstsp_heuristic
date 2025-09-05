
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

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

maxIteration = 200000

## END CONFIG

class Truck_Subroutes:
    def __init__(self, subroute):
        self.route = subroute
        self.drone_route = None     # the drone route working parallel with this truck subroute

    def assign_drone_route(self, drone_route):
        self.drone_route = drone_route


# Main algorithm
def fstsp_heuristic(numnodes, parcel_weight, delta_T, delta_D, tsp_solver='ortools'):
    """
    Input: the adjaciency matrix of truck and the adjacency matrix of drone

    Return: truck_route, time_to_node (truck), drone_routes, timespan

    """
    
    # set of all customers
    C = list(range(0, numnodes))

    if tsp_solver == 'ortools':
        [truck_route, time_to_node] = solveTSP(delta_T)
    elif tsp_solver == 'twoopt':
        truck_route, time_to_node = two_opt(delta_T)
    else:
        raise ValueError("FSTSP_heuristic: Invalid choose for tsp solver!")

    # print("Initial tsp route: ", truck_route)

    # set of uav eligible customers
    Cprime = get_uav_eligible_cus(parcel_weight)

    # print("Initial Cprime: ", Cprime)

    # partitioning truck route to the subroutes by the nodes of launching and retriving drone
    truck_subroutes = [Truck_Subroutes(truck_route)]

    # drone route is set of travel arc = (launching node, delivery node, retrieve node)
    drone_routes = []

    time_to_node = recalc_time(truck_route, drone_routes, delta_T, delta_D)
    # print("Initial truck TSP route: ", truck_route, "\t Timespan: ",time_to_node[-1])

    max_saving = 0

    for _ in range(maxIteration):

        # for perform the update function
        serve_by_drone = None         # False means serve by truck, True means serve by drone
        i_star = None
        k_star = None

        for j in Cprime:
            # The saving associated with removing node j from its position in the truck tour
            savings = calc_savings(j, delta_T, delta_D, time_to_node, truck_route, drone_routes)

            for subroute in truck_subroutes:
                if is_uav_in_subroute(subroute):
                    # calculating the cost of inserting node j to truck route
                    new_j_star, new_i_star, new_k_star, new_max_saving, new_serve_by_drone = calc_cost_truck(j, time_to_node, truck_route, subroute.route, delta_T, savings, max_saving)

                else:
                    # calculating the cost of inserting node j to drone operation
                    new_j_star, new_i_star, new_k_star, new_max_saving, new_serve_by_drone = calc_cost_uav(j, time_to_node, truck_route, subroute.route, delta_T, delta_D, savings, max_saving)
                
                # update max_saving
                if new_max_saving > max_saving:
                    max_saving = new_max_saving
                    i_star = new_i_star
                    j_star = new_j_star
                    k_star = new_k_star
                    serve_by_drone = new_serve_by_drone

        if max_saving > 0:
            # print(f"Updating with: i_star: {i_star}, j_star: {j_star}, k_star: {k_star}, serve_by_drone: {serve_by_drone}, max_saving: {max_saving}")

            # perfom_update()
            if serve_by_drone:
                # the drone is node assigned to i* -> j* -> k*
                # first remove i* j* k* from Cprime
                if i_star in Cprime: Cprime.remove(i_star)
                if j_star in Cprime: Cprime.remove(j_star)
                if k_star in Cprime: Cprime.remove(k_star)

                # Append a drone arc to drone_routes
                drone_routes.append([i_star, j_star, k_star])


                # Remove j* from truck_route and truck_subroutes then update the truck arrival time array time_to_node
                truck_route.remove(j_star)

                for subroute in truck_subroutes:
                    if j_star in subroute.route:
                        subroute.route.remove(j_star)

                # Append a new truck subroute that start from i* and ends at k*
                for subroute in truck_subroutes[:]:  # iterate over a shallow copy
                    if i_star in subroute.route and k_star in subroute.route:
                        i = subroute.route.index(i_star)
                        k = subroute.route.index(k_star)

                        # Ensure i comes before k
                        if i <= k:
                            subroute1 = Truck_Subroutes(subroute.route[:i+1])
                            subroute2 = Truck_Subroutes(subroute.route[i:k+1])
                            subroute3 = Truck_Subroutes(subroute.route[k:])

                            subroute2.assign_drone_route([i_star, j_star, k_star])

                            if len(subroute1.route)>1: truck_subroutes.append(subroute1)
                            if len(subroute2.route)>1: truck_subroutes.append(subroute2)
                            if len(subroute3.route)>1: truck_subroutes.append(subroute3)

                            truck_subroutes.remove(subroute)

                time_to_node = recalc_time(truck_route, drone_routes, delta_T, delta_D)

            else: # serve by truck

                # Update truck_subroutes
                for idx in range(len(truck_subroutes)):
                    subroute = truck_subroutes[idx]
                    if j_star in subroute.route:
                        subroute.route.remove(j_star)
                    if i_star in subroute.route:
                        i = subroute.route.index(i_star)
                        subroute.route.insert(i+1, j_star)

                # update truck route
                truck_route.remove(j_star)
                i = truck_route.index(i_star)
                truck_route.insert(i+1, j_star)

                # update time array
                time_to_node = recalc_time(truck_route, drone_routes, delta_T, delta_D)


            # reset max_savings value
            max_saving = 0
        else:
            break

    # Re-check time array
    time_to_node = recalc_time(truck_route, drone_routes, delta_T, delta_D)

    # Calculating timespan
    timespan = time_to_node[-1]

    for drone_arc in drone_routes:
            if drone_arc[2] == 0:   # drone going back to depot itself
                launching_time = time_to_node[truck_route.index(drone_arc[0])]

                return_time = (launching_time +
                               drone_launch_time + 
                               delta_D[drone_arc[0], drone_arc[1]] +
                               drone_service_time +
                               delta_D[drone_arc[1], drone_arc[2]] +
                               drone_recover_time)
                
                if time_to_node[-1] < return_time:   # drone go back to depot after the truck
                    timespan = return_time

    return truck_route, time_to_node, drone_routes, timespan


def get_uav_eligible_cus(parcel_weight):
    Cprime = []

    for i in range(len(parcel_weight)):
        if parcel_weight[i] <= drone_capacity and parcel_weight[i] > 0:
            Cprime.append(i)
    
    return Cprime

def calculate_route_distance(route, dist_matrix):
    return sum(dist_matrix[route[i], route[i + 1]] for i in range(len(route) - 1)) + dist_matrix[route[-1], route[0]]

def compute_time_to_each_node(route, dist_matrix):
    time_to_node = [0]  # starting at node 0, time = 0
    current_time = 0
    for i in range(1, len(route)):
        current_time += dist_matrix[route[i - 1], route[i]]
        time_to_node.append(current_time)
    return time_to_node

def two_opt(dist_matrix, max_iter=500):
    n = len(dist_matrix)
    route = list(range(n))  # initial route: [0, 1, ..., n-1]
    route.append(0)
    best_distance = calculate_route_distance(route, dist_matrix)

    for _ in range(max_iter):
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n):
                if j - i == 1: continue
                new_route = route[:i] + route[i:j][::-1] + route[j:]
                new_distance = calculate_route_distance(new_route, dist_matrix)
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance
                    improved = True
        if not improved:
            break

    time_to_node = compute_time_to_each_node(route, dist_matrix)
    return route, time_to_node

def solveTSP(distance_matrix):

    """
        vehicle_velocity    to calculate time traveling in an edge
        service_time        is the time spend to drop the package    
    """
    
    num_vehicles = 1
    depot = 0

    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        num_vehicles,
        depot
    )

    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        print("No solution found!")
        return
    

    # Init array for output
    tsp_route = []
    time_to_node = []

    if solution:
        index = routing.Start(0)

        previous_node = manager.IndexToNode(index)

        while not routing.IsEnd(index):
            next_node = manager.IndexToNode(index)
            # append node to tsp route
            tsp_route.append(next_node)

            previous_index = index
            previous_node = next_node

            index = solution.Value(routing.NextVar(index))
            route_distance = routing.GetArcCostForVehicle(previous_index, index, 0)

            travel_time = route_distance/truck_velocity

            # add the service time of next node to calculate the time, comming to current node
            if previous_node != depot:
                travel_time += truck_service_time
            
            if len(time_to_node) > 0: 
                last_node_time = time_to_node[-1]
            else:
                last_node_time = 0

            time_to_node.append(last_node_time+travel_time)
        
        # turn back to depot
        next_node = manager.IndexToNode(index)
        tsp_route.append(next_node)
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)

        travel_time = route_distance/truck_velocity

        # add the service time of next node to calculate the time, comming to current node
        if previous_node != depot:
            travel_time += truck_service_time

        last_node_time = time_to_node[-1]
        time_to_node.append(last_node_time+travel_time)

    return tsp_route, time_to_node


def calc_savings(node_j, delta_T, delta_D, time_array, truck_route, drone_routes):

    """ The saving associated with removing node j from its position in the truck tour"""

    savings = 0

    for idx in range(1, len(truck_route)-1):
        if node_j == truck_route[idx]:
            preceding_node = truck_route[idx-1]
            successor_node = truck_route[idx+1]

            savings = delta_T[preceding_node,node_j] + delta_T[node_j, successor_node] - delta_T[preceding_node,successor_node] - truck_service_time

            # check if node j is currently in a truck subroute paired with the UAV
            prev_j = truck_route[:idx]
            afte_j = truck_route[idx+1:]

            for drone_arc in drone_routes:
                a = drone_arc[0]
                drone_cust = drone_arc[1]
                b = drone_arc[2]

                if a in prev_j and b in afte_j:
                    # calculate the t_prime[b] the time truck arrive to b if j is removed from the truck node
                    pos_b = truck_route.index(b)
                    t_prime_b = time_array[pos_b] - savings

                    t_a = time_array[truck_route.index(a)]

                    drone_a_to_b = t_a + delta_D[a,drone_cust] + delta_D[drone_cust,b] + drone_recover_time + drone_launch_time + drone_service_time

                    # new saving value
                    savings = min(savings, (t_prime_b - drone_a_to_b))

    return savings


def is_uav_in_subroute(subroute):

    """ Check whether a drone working parallel with the truck in this subtour
        
        Return a boolen value   
    """

    result = False

    # for drone_arc in drone_routes:
    #     a = drone_arc[0]
    #     cust = drone_arc[1]
    #     b = drone_arc[2]

    #     if a in subroute and b in subroute:
    #         result = True

    if subroute.drone_route is not None:
        result = True

    return result

def calc_cost_truck(node_j, time_to_node, truck_route, subroute, delta_T, savings, max_saving):

    """ Considering insert node j to truck route.
    
        Return: node of changing
                the precessor node
                the successor node
                the saving                          """
    
    # output
    preced_node = None
    succed_node = None
    served_by_drone = None

    a = subroute[0]
    b = subroute[-1]

    for i in range(len(subroute)-1):
        node1 = subroute[i]
        node2 = subroute[i+1]

        if node1 == node_j or node2 == node_j:
            continue

        # try to insert node_j to between i and i+1
        cost = delta_T[node1, node_j] + delta_T[node_j, node2] - delta_T[node1, node2]

        if cost < savings:  # insert node_j to this position is beter than last position
            t_b = time_to_node[truck_route.index(b)]
            t_a = time_to_node[truck_route.index(a)]

            # Can the drone assigned to this subroute still feasibly fly
            if (t_b - t_a + cost) < drone_battery:
                if (savings - cost) > max_saving:
                    max_saving = savings - cost
                    preced_node = node1
                    succed_node = node2
                    served_by_drone = False

    # print(f'In calc_cost_truck() function: max_savings: {max_saving}, i: {preced_node}, j: {node_j}, k: {succed_node}, served_by_drone: {served_by_drone}')

    return node_j, preced_node, succed_node, max_saving, served_by_drone


def calc_cost_uav(node_j, time_to_node, truck_route, subroute, delta_T, delta_D, savings, max_saving):

    """ This subroute is not associate with a drone visit. Then try to assign one drone operation here."""

    # output
    preced_node = None
    succed_node = None
    served_by_drone = None

    for i in range(len(subroute)-1):
        for k in range(i+1, len(subroute)):

            node_i = subroute[i]
            node_k = subroute[k]

            if node_i == node_j or node_k == node_j:
                continue

            if (delta_D[node_i,node_j] + delta_D[node_j,node_k] + drone_launch_time + drone_recover_time + drone_service_time) < drone_battery:
                
                # find t_prime_k, the truck arrival time to node k if j is removed from the truck's route
                t_prime_k = time_to_node[truck_route.index(node_k)]

                for r in range(truck_route.index(node_k)):
                    # t_prime_k only change when the removed node_j is before node k in truck's route
                    if node_j == truck_route[r]: 
                        preced_j = truck_route[r-1]
                        succed_j = truck_route[r+1]
                        t_prime_k = t_prime_k - delta_T[preced_j,node_j] - delta_T[node_j,succed_j] + delta_T[preced_j,succed_j] - truck_service_time
                
                t_i = time_to_node[truck_route.index(node_i)]

                drone_travel_time = max((t_prime_k - t_i),(delta_D[node_i,node_j]+delta_D[node_j,node_k]+drone_launch_time+drone_recover_time+drone_service_time))    # sychronize time arrives
                truck_travel_time = t_prime_k - t_i
                cost = max(0, (drone_travel_time-truck_travel_time))
                
                if (savings - cost) > max_saving:
                    max_saving = savings - cost
                    preced_node = node_i
                    succed_node = node_k
                    served_by_drone = True
    
    return node_j, preced_node, succed_node, max_saving, served_by_drone


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


def test():
    distance_matrix = [
    [0,  2,  9, 10, 7],
    [1,  0,  6,  4, 3],
    [15, 7,  0,  8, 3],
    [6,  3, 12,  0, 5],
    [10, 4,  8,  6, 0]
    ]

    route, time = solveTSP(distance_matrix, truck_velocity, truck_service_time)

    print('route: ', route)
    print('time to each node: ', time)

if __name__ == "__main__":
    test()
