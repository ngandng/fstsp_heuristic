
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

maxIteration = 20000

## END CONFIG


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


    # recalculation time to node
    returned_time = [0]
    for j in range(1, len(tsp_route)):
        node_j = tsp_route[j]
        prev_node = tsp_route[j-1]

        travel_time = distance_matrix[prev_node,node_j]

        if prev_node != 0:  # add service time of previous node
            travel_time += truck_service_time

        arrival_time = returned_time[-1] + travel_time
        returned_time.append(round(float(arrival_time), 2))

    timespan = returned_time[-1]

    return tsp_route, returned_time, timespan