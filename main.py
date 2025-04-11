
import time

from fstsp_heuristic import fstsp_heuristic
from utils import process_location_data

if __name__ == "__main__":

    testproblem = 'my_test'

    locations, num_nodes, parcel_weight, delta_T, delta_D = process_location_data(testproblem)

    # test
    print('locations: ', locations)
    print('num_nodes: ', num_nodes)
    print('parcel_weight: ', parcel_weight)
    print('\ndelta_T: ', delta_T, '\n')
    print('delta_D: ', delta_D, '\n')

    start_time = time.time()

    truck_route, time_array, drone_routes = fstsp_heuristic(locations, num_nodes, parcel_weight, delta_T, delta_D)

    end_time = time.time()

    print('COmputation time: ', end_time-start_time)
    print(f'Solution: \nTruck route: {truck_route}\nTruck time to node: {time_array}\nDrone routes: {drone_routes}')
