
import time
import argparse

from algorithms.fstsp_heuristic import fstsp_heuristic
from algorithms.cp_aco import cp_aco
from utils import process_location_data
from plot import plot_map

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run FSTSP Heuristic Algorithm")
    parser.add_argument('--test_instance', type=str, default='my_test',
                        help='Path to the test instance file')
    
    parser.add_argument('--algorithm', type=str, default='fstsp_heuristic',
                        help='Type of algorithm')

    args = parser.parse_args()

    testproblem = args.test_instance

    locations, num_nodes, parcel_weight, delta_T, delta_D = process_location_data(testproblem)

    # Print the input
    # print('locations: ', locations)
    # print('num_nodes: ', num_nodes)
    # print('parcel_weight: ', parcel_weight)
    # print('\ndelta_T: ', delta_T, '\n')
    # print('delta_D: ', delta_D, '\n')

    start_time = time.time()

    if args.algorithm == 'fstsp_heuristic':
        truck_route, time_array, drone_routes, timespan = fstsp_heuristic(num_nodes, parcel_weight, delta_T, delta_D)
    elif args.algorithm == 'cp_aco':
        truck_route, time_array, drone_routes, timespan = cp_aco(num_nodes, parcel_weight, delta_T, delta_D)

    end_time = time.time()

    print('\n\nComputation time: ', end_time-start_time)
    print(f'\n\nSolution: \nTruck route: {truck_route}\n\nTruck time to node: {time_array}\n\nDrone routes: {drone_routes} \n\nTimespan: {timespan}')
    print(f'\n\nNumber of drone customer: {len(drone_routes)} \nNumber of truck customer: {len(truck_route)-2}')

    # Plot
    plot_map(locations, truck_route, drone_routes)
