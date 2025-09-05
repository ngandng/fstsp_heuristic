"""
    Reference:  Bouman, Paul, Niels Agatz, and Marie Schmidt. 
                "Dynamic programming approaches for the traveling salesman problem with drone." 
                Networks (2018).
"""

import itertools
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
    P = {}

    all_sets = []
    for s in range(2, n + 1):
        for subset in itertools.combinations(range(n), s):
            sset = frozenset(subset)
            all_sets.append(sset)
    all_nodes = frozenset(range(n))


    ### FIRST PASS: for every truck path that start at v and end at w
    # Base cases iteration
    for v in range(n):
        for w in range(v+1, n):
            if v == w:
                continue

            D_T[(frozenset([v, w]), v, w)] = delta_T[v][w]
            D_T[(frozenset([v, w]), w, v)] = delta_T[v][w]
            P[(frozenset([v, w]), v, w)] = v
            P[(frozenset([v, w]), w, v)] = w

    for S in all_sets:
        print('Subset: ', S)
        for v in S:             # Note: v and w has to be in S
            for w in S:
                print('v = ',v, 'w = ', w)
                if v == w:
                    continue
                # compute best path v→...→w covering S
                best_cost, best_prev = float("inf"), None
                for u in S:
                    print('u = ', u)
                    if u == w or u==v:
                        continue
                    if (S - {w}, v, u) not in D_T:
                        continue
                    cost = D_T[(S - {w}, v, u)] + delta_T[u][w]
                    print('cost = ', cost)
                    if cost < best_cost:
                        best_cost, best_prev = cost, u
                if best_cost < float("inf"):
                    D_T[(S, v, w)] = best_cost
                    P[(S, v, w)] = best_prev

    print('\nD: ', D_T)
    print('\nP: ', P)



    ### SECOND PASS
    D_OP = {}
    for subset in all_sets:
        # print('subset: ', subset)
        for v in subset:
            for w in subset:
                if v == w:
                    continue
                best = D_T[(S, v, w)]
                for d in S - {v, w}:
                    truck_time = D_T[(S - {d}, v, w)]
                    drone_time = delta_D[v][d] + delta_D[d][w]
                    total_time = max(truck_time, drone_time)
                    best = min(best, total_time)
                D_OP[(S, v, w)] = best

    print('\nD_OP: ', D_OP)



    ### THIRD PASS
    v_0 = 0     # defining depot
    D = {}
    D[(None, v_0)] = 0
    
    for U in all_sets:
        V = all_nodes - subset
        for r in range(1, len(V) + 1):   # size of subset
            for T in itertools.combinations(V, r):



    # Close the tour: return to start
    start = 0
    best_cost, last = min(
        (D_T[(all_nodes, start, w)] + delta_T[w][start], w)
        for w in range(n) if w != start
    )
    
    # Reconstruct path
    path = [start]
    S = all_nodes
    w = last
    # print('path: ', path, 'S: ', S)
    while True:
        path.append(w)
        u = P.get((S, start, w), None)
        # print('u: ', u, 'w: ', w)
        if u is None:
            break
        S = S - {w}
        # print('path: ', path, 'S: ', S)
        w = u
        if w == start:
            break
    path.append(start)
    
    return path, best_cost


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
    start_location = 0
    locations = list(range(len(cost_matrix)))
    numnodes = 4
    path, min_cost = dp_tspd(numnodes, None, cost_matrix, drone_matrix)
    print("Shortest path:", path)
    print("Minimum cost:", min_cost)
