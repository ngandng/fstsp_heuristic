"""
    Reference:  Bouman, Paul, Niels Agatz, and Marie Schmidt. 
                "Dynamic programming approaches for the traveling salesman problem with drone." 
                Networks (2018).
"""

import math
import itertools
from collections import defaultdict
# from config import config

# ### CONFIG:
# drone_velocity = config['drone_velocity']
# drone_battery = config['drone_battery']
# drone_launch_time = config['drone_launch_time']
# drone_recover_time = config['drone_recover_time']
# drone_service_time = config['drone_service_time']
# drone_capacity = config['drone_capacity']

# truck_velocity = config['truck_velocity']
# truck_service_time = config['truck_service_time']

## END CONFIG



# Main algorithm
def dp_tspd(numnodes, parcel_weight, delta_T, delta_D):
    """
    Input: the traveling matrix of truck and the traveling matrix of drone

    Return: truck_route, time_to_node (truck), drone_routes, timespan

    """

    ### INIT
    n = numnodes

    D_T = {}
    # P = {}

    all_nodes = frozenset(range(n))


    ### FIRST PASS: for every truck path that start at v and end at w
    # Base cases iteration
    for v in range(n):
        for w in range(v+1, n):
            if v == w:
                continue
            if w != 0:
                D_T[(frozenset([v, w]), v, w)] = delta_T[v][w]
            if v != 0:
                D_T[(frozenset([v, w]), w, v)] = delta_T[v][w]

    # TODO: Fix the case of D_T(frozenset({0, 1, 2}), 2, 1): 25 -> can not do 2->0->1
    for s in range(2, n + 1):
        for subset in itertools.combinations(range(n), s):  # all possible subsets of size i from the set range(n)
            S = frozenset(subset)
            # print('Subset: ', S)
            for v in S:             # Note: v and w has to be in S
                for w in S:
                    # print('v = ',v, 'w = ', w)
                    if v == w  or w==0:
                        continue
                    # compute best path v→...→w covering S
                    best_cost, best_prev = float("inf"), None
                    for u in S:
                        # print('u = ', u)
                        if u == w or u==v:
                            continue
                        if (S - {w}, v, u) not in D_T:
                            continue
                        cost = D_T[(S - {w}, v, u)] + delta_T[u][w]
                        # print('cost = ', cost)
                        if cost < best_cost:
                            best_cost, best_prev = cost, u
                    if best_cost < float("inf"):
                        D_T[(S, v, w)] = best_cost
                        # P[(S, v, w)] = best_prev

    # print('\nD_T: ', D_T)
    # print('\nP: ', P)



    ### SECOND PASS
    D_OP = {}
    P_D = {}    # which node should be assigned for drone in (S,v,w)
    for s in range(1, n+1):
        for subset in itertools.combinations(range(n), s):  # all possible subsets of size i from the set range(n)
            subset = frozenset(subset)
            # print('subset: ', subset)

            if len(subset) == 1:
                v = next(iter(subset)) 
                D_OP[(subset, v, v)] = 0
                continue

            for v in subset:
                for w in subset:
                    if v == w or w==0:
                        continue
                    # best = D_T[(subset, v, w)]
                    best = D_T.get((subset, v, w), math.inf)
                    if best == math.inf:
                        continue
                    best_d = 0
                    for d in subset - {v, w}:   # for every node drone can take
                        truck_time = D_T[(subset - {d}, v, w)]
                        drone_time = delta_D[v][d] + delta_D[d][w]
                        total_time = max(truck_time, drone_time)
                        if total_time < best:
                            best = total_time
                            best_d = d
                    D_OP[(subset, v, w)] = best
                    if best_d != 0:
                        P_D[(subset, v, w)] = best_d

    # print('\nD_OP: ', D_OP)
    # print('\nP_D: ', P_D)



    ### THIRD PASS
    v_0 = 0     # defining depot
    D = defaultdict(lambda: math.inf)
    P_T = {}    # which node will be in the next sub_routes
    P_OP = {}   # nodes can be assigned for truck in this sub-routes
    P_OPD = {}  # nodes will be assigned for drone in this sub_routes

    # base case
    D[(frozenset(), v_0)] = 0   # base case
    for (u,) in itertools.combinations(all_nodes, 1):
        D[(frozenset([u]), u)] = delta_T[v_0][u]

    
    for i in range(1, n + 1):   # |U| = i
        for U in itertools.combinations(all_nodes, i):
            U = set(U)
            # print('U: ', U)
            V_minus_U = all_nodes - U
            # all subsets T of V\U (including empty)
            for r in range(0, len(V_minus_U) + 1):
                for T in itertools.combinations(V_minus_U, r):
                    T = set(T)
                    # print('T: ', T)
                    for u in U:
                        for w in all_nodes:
                            prev_cost = D[(frozenset(U), u)]
                            if prev_cost == math.inf:
                                continue
                            op_cost = D_OP.get((frozenset(T | {u}), u, w), math.inf)
                            z = prev_cost + op_cost
                            newS = frozenset(U | {u, w} | T)
                            # print('S: ', newS, '\tDrone_op set: ', T | {u}, '\t(u,w)=',u,w, '\tz = ', z, '\tD[S,w] = ', D[(newS, w)])
                            if z < D[(newS, w)]:
                                # print("Updated D[{%s},%d] = %d" % (newS, w, z))
                                # print('U: ', U, 'T: ', T)
                                # print('S: ', newS, '\tDrone_op set: ', T | {u}, '\t(u,w)=',u,w, '\tz = ', z, '\tD[S,w] = ', D[(newS, w)])
                                D[(newS, w)] = z
                                drone_nodes = P_D.get((frozenset(T | {u}), u, w), None)
                                P_T[(newS, w)] = u
                                if drone_nodes != None:
                                    P_OP[(newS, w)] = frozenset(T - {w, drone_nodes})
                                    P_OPD[(newS, w)] = drone_nodes
    # print('\nD: ', D)
    # print('\nP_T: ', P_T)
    # print('\nP_OP: ', P_OP)


    # Close the tour: return to start
    start = 0
    best_cost, last = min(
        (D[(all_nodes, w)] + delta_T[w][start], w)
        for w in range(n) if w != start
    )
    
    # Reconstruct path
    path_truck = [start]
    path_drone = []

    S = all_nodes
    w = last
    # print('path: ', path, 'S: ', S)
    while True:
        path_truck.append(w)
        u = P_T.get((S, w), None)
        u_op = P_OPD.get((S, w), None)
        # print('Truck: ', path_truck, '\nDrone: ', path_drone)
        # print('S: ', S, 'w: ', w, 'u: ', u, 'u_op: ', u_op, '\n')
        if u is None:
            break
        if u_op:
            truck_tasks = P_OP.get((S, w), None)
            S = S - {u_op} - truck_tasks
            if truck_tasks: path_truck += list(truck_tasks)
            path_drone.append((w, u_op, u))
        
        S = S - {w}
        # print('path: ', path, 'S: ', S)
        w = u
        if w == start:
            # print('Truck: ', path_truck, '\nDrone: ', path_drone)
            # print('S: ', S, 'w: ', w, 'u: ', u, 'u_op: ', u_op, '\n')
            break
    path_truck.append(start)
    
    return path_truck, None, path_drone, best_cost


# Testing
if __name__ == "__main__":
    # Example cost matrix (symmetric)
    # cost_matrix = [
    #     [0, 10, 15, 20],
    #     [10, 0, 35, 25],
    #     [15, 35, 0, 30],
    #     [20, 25, 30, 0]
    # ]
    # drone_matrix = [
    #     [0, 5, 8, 10],
    #     [5, 0, 18, 12],
    #     [8, 18, 0, 15],
    #     [10, 12, 15, 0]
    # ]

    cost_matrix = [
        [0, 10, 15, 20, 25, 30],
        [10, 0, 35, 25, 17, 28],
        [15, 35, 0, 30, 22, 26],
        [20, 25, 30, 0, 24, 18],
        [25, 17, 22, 24, 0, 16],
        [30, 28, 26, 18, 16, 0]
    ]
    drone_matrix = [
        [0, 5, 8, 10, 12, 14],
        [5, 0, 18, 12, 10, 11],
        [8, 18, 0, 15, 9, 13],
        [10, 12, 15, 0, 11, 7],
        [12, 10, 9, 11, 0, 6],
        [14, 11, 13, 7, 6, 0]
    ]

    start_location = 0
    locations = list(range(len(cost_matrix)))
    numnodes = 6
    path_truck, _, path_drone, best_cost = dp_tspd(numnodes, None, cost_matrix, drone_matrix)

    print("\n\nTruck path:", path_truck)
    print("Drone path: ", path_drone)
    print("Minimum cost:", best_cost)
    
