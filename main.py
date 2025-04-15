
import time
import argparse

from fstsp_heuristic import fstsp_heuristic
from cp_aco import cp_aco
from utils import process_location_data

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run FSTSP Heuristic Algorithm")
    parser.add_argument('--test_instance', type=str, default='20170608T122008595748',
                        help='Path to the test instance file')

    args = parser.parse_args()

    testproblem = args.test_instance

    locations, num_nodes, parcel_weight, delta_T, delta_D = process_location_data(testproblem)

    # test
    print('locations: ', locations)
    print('num_nodes: ', num_nodes)
    print('parcel_weight: ', parcel_weight)
    print('\ndelta_T: ', delta_T, '\n')
    print('delta_D: ', delta_D, '\n')

    start_time = time.time()

    # truck_route, time_array, drone_routes = fstsp_heuristic(num_nodes, parcel_weight, delta_T, delta_D)
    truck_route, time_array, drone_routes, timespan = cp_aco(num_nodes, parcel_weight, delta_T, delta_D)

    end_time = time.time()

    print('\n\nComputation time: ', end_time-start_time)
    print(f'\n\nSolution: \nTruck route: {truck_route}\n\nTruck time to node: {time_array}\n\nDrone routes: {drone_routes} \n\nTimespan: {timespan}')
