"""
    Reference:  Bouman, Paul, Niels Agatz, and Marie Schmidt. 
                "Dynamic programming approaches for the traveling salesman problem with drone." 
                Networks (2018).
"""

import math
import itertools
from collections import defaultdict
# from config import config

from algorithms.fstsp_heuristic import recalc_time as objfcn

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
    end_node = numnodes     # the last index N+1 is for the depot
    n = numnodes + 1        # hence number of node is now N+1    

    all_nodes = frozenset(range(n))


    ### FIRST PASS: for every truck path that start at v and end at w
    D_T = {}
    # P = {}

    for v in range(n): # Base cases iteration
        for w in range(v+1, n):
            if v == w:
                continue
            if w != 0 and v != end_node:
                if w == end_node:
                    D_T[(frozenset([v, w]), v, w)] = delta_T[v][0]
                else:
                    D_T[(frozenset([v, w]), v, w)] = delta_T[v][w]
            if v != 0 and w != end_node:
                if v == end_node:
                    D_T[(frozenset([v, w]), w, v)] = delta_T[w][0]
                else:
                    D_T[(frozenset([v, w]), w, v)] = delta_T[w][v]

    
    for s in range(2, n + 1):
        for subset in itertools.combinations(range(n), s):
            S = frozenset(subset)

            # If 0 is in S, v must be 0; otherwise v can be anything in S.
            if 0 in S:
                v_candidates = {0}
            else:
                v_candidates = S

            # If end_node is in S, w must be end_node; otherwise w can be anything in S.
            if end_node in S:
                w_candidates = {end_node}
            else:
                w_candidates = S

            for v in v_candidates:
                for w in w_candidates:
                    if v == w:
                        continue

                    best_cost, best_prev = float("inf"), None
                    for u in S:
                        if u in {v, w}:
                            continue
                        if (S - {w}, v, u) not in D_T:
                            continue
                        cost = D_T[(S - {w}, v, u)] + (delta_T[u][0] if w == end_node else delta_T[u][w])
                        if cost < best_cost:
                            best_cost, best_prev = cost, u

                    if best_cost < float("inf"):
                        D_T[(S, v, w)] = best_cost
                        # P[(S, v, w)] = best_prev

    # print('\nD_T: ', D_T)
    # print('\nP: ', P)



    ### SECOND PASS: which drone node is best for (S,v,w) start at v, end at w and cover every node in S\{d} with d is drone node
    D_OP = {}
    P_D = {}    # which node should be assigned for drone in (S,v,w)
    for s in range(1, n+1):
        for subset in itertools.combinations(range(n), s):  # all possible subsets of size i from the set range(n)
            S = frozenset(subset)

            if len(S) == 1:
                v = next(iter(S)) 
                D_OP[(S, v, v)] = 0
                continue

            for v in S:
                for w in S:
                    # if v == w or w==0:
                    if v == w:
                        continue
                    best = D_T.get((S, v, w), math.inf)
                    if best == math.inf:
                        continue
                    best_d = 0
                    for d in S - {v, w}:   # for every node drone can take
                        truck_time = D_T[(S - {d}, v, w)]
                        drone_time = delta_D[v][d] + (delta_D[d][w] if w != end_node else delta_D[d][0])
                        total_time = max(truck_time, drone_time)
                        if total_time < best:
                            best = total_time
                            best_d = d
                    D_OP[(S, v, w)] = best
                    if best_d != 0:
                        P_D[(S, v, w)] = best_d

    # print('\nD_OP: ', D_OP)
    # print('\nP_D: ', P_D)



    ### THIRD PASS: cost of a subproblem D(S,w), start at v_0=0, cover all nodes in S and end at w \in S
    D = defaultdict(lambda: math.inf)
    P_T = {}    # which end node will be for the next sub_routes
    P_OP = {}   # nodes can be assigned for truck in this sub_routes
    P_OPD = {}  # nodes will be assigned for drone in this sub_routes

    # base case
    D[(frozenset(), 0)] = 0   # base case
    for (u,) in itertools.combinations(all_nodes, 1):
        D[(frozenset([u]), u)] = (delta_T[0][u] if u != end_node else 0)

    
    for i in range(1, n+1):   # |U| = i
        for U in itertools.combinations(all_nodes, i):
            U = set(U)
            # print('U: ', U)
            V_minus_U = all_nodes - U
            
            for r in range(0, len(V_minus_U) + 1):
                for T in itertools.combinations(V_minus_U, r): # all subsets T of V\U (including empty)
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

                            if z < D[(newS, w)]:
                                D[(newS, w)] = z
                                drone_nodes = P_D.get((frozenset(T | {u}), u, w), None)
                                P_T[(newS, w)] = u
                                if drone_nodes != None:
                                    P_OP[(newS, w)] = frozenset(T - {w, drone_nodes})
                                    P_OPD[(newS, w)] = drone_nodes
    # print('\nD: ', D)
    # print('\nP_T: ', P_T)
    # print('\nP_OP: ', P_OP)


    ### EXTRACT SOLUTION
    start = 0
    S = all_nodes
        
    # Reconstruct path
    path_truck = []
    path_drone = []
    
    best_cost = D[(all_nodes, end_node)]

    w = end_node
    while True:
        path_truck.append(w if w != end_node else start)
        u = P_T.get((S, w), None)
        u_op = P_OPD.get((S, w), None)
        if u is None:
            break
        if u_op:
            truck_tasks = P_OP.get((S, w), None)
            S = S - {u_op} - truck_tasks
            if truck_tasks: path_truck += list(truck_tasks)
            path_drone.append((w, u_op, u) if w!=end_node else (0, u_op, u))
        
        S = S - {w}
        w = u
        if w == start:
            break
    path_truck.append(start)

    time = objfcn(path_truck, path_drone, delta_T, delta_D)
    print('Calculated time: ', time)
    best_cost = time[-1]
    
    return path_truck, None, path_drone, best_cost
    


# Testing
if __name__ == "__main__":
    # Example cost matrix (symmetric)
    cost_matrix = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]
    drone_matrix = [
        [0, 5, 8, 10],
        [5, 0, 18, 12],
        [8, 18, 0, 15],
        [10, 12, 15, 0]
    ]

    # cost_matrix = [
    #     [0, 10, 15, 20, 25, 30],
    #     [10, 0, 35, 25, 17, 28],
    #     [15, 35, 0, 30, 22, 26],
    #     [20, 25, 30, 0, 24, 18],
    #     [25, 17, 22, 24, 0, 16],
    #     [30, 28, 26, 18, 16, 0]
    # ]
    # drone_matrix = [
    #     [0, 5, 8, 10, 12, 14],
    #     [5, 0, 18, 12, 10, 11],
    #     [8, 18, 0, 15, 9, 13],
    #     [10, 12, 15, 0, 11, 7],
    #     [12, 10, 9, 11, 0, 6],
    #     [14, 11, 13, 7, 6, 0]
    # ]

    start_location = 0
    locations = list(range(len(cost_matrix)))
    numnodes = len(cost_matrix)
    path_truck, _, path_drone, best_cost = dp_tspd(numnodes, None, cost_matrix, drone_matrix)
    print("\n\nNumber of node:", numnodes)
    print("Truck path:", path_truck)
    print("Drone path: ", path_drone)
    print("Minimum cost:", best_cost)
    
