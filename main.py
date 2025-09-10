
import time
import argparse
import os

import numpy as np
import pandas as pd

from algorithms.fstsp_heuristic import fstsp_heuristic
from algorithms.cp_aco import cp_aco
from algorithms.tsp import solveTSP
from algorithms.dp import dp_tspd


from utils import process_location_data
from plot import plot_map

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run FSTSP Heuristic Algorithm")
    parser.add_argument('--test_instance', type=str, default='my_test',
                        help='Path to the test instance file')
    
    parser.add_argument('--algorithm', type=str, default='fstsp_heuristic',
                        help='Type of algorithm')

    args = parser.parse_args()


    # some values
    objective_values = []
    ep_time = []
    results = {}

    will_save = False

    # ===> TEST WITH PARSE INSTANCE
    # testproblem = args.test_instance
    # total_test_episodes = 1

    # ===> TEST WITH A FOLDER
    # city_name = 'Incheon_small'
    # city_name = 'Incheon_big'
    # city_name = "Ulsan_Namgu"
    # num_nodes = 26
    # folder_name = 'data/new_test/' + str(city_name) +'/' + str(num_nodes) + '/'
    # problem_names = [f.name for f in os.scandir(folder_name) if f.is_dir()]
    # problem_name = folder_name + problem_names[0]
    # total_test_episodes = min(len(problem_names), 1000)

    # ===> TEST WITH SAMPLE DATA
    folder_name = 'my_test'
    city_name = 'my_test'
    total_test_episodes = 1

    for ep in range(total_test_episodes):
        if total_test_episodes == 1:
            testproblem = folder_name
        else:
            testproblem = folder_name + problem_names[ep]

        locations, num_nodes, parcel_weight, delta_T, delta_D = process_location_data(testproblem)

        # Print the input
        # print('locations: ', locations)
        # print('num_nodes: ', num_nodes)
        # print('parcel_weight: ', parcel_weight)
        # print('\ndelta_T: ', delta_T, '\n')
        # print('delta_D: ', delta_D, '\n')

        tsp_solver = 'ortools' # or twoopt

        start_time = time.time()

        if args.algorithm == 'fstsp_heuristic':
            truck_route, time_array, drone_routes, timespan = fstsp_heuristic(num_nodes, parcel_weight, delta_T, delta_D, tsp_solver=tsp_solver)
        elif args.algorithm == 'cp_aco':
            truck_route, time_array, drone_routes, timespan = cp_aco(num_nodes, parcel_weight, delta_T, delta_D)
        elif args.algorithm == 'tsp':
            truck_route, time_array, timespan = solveTSP(delta_T)
            drone_routes = []
        elif args.algorithm == 'dp':
            truck_route, time_array, drone_routes, timespan = dp_tspd(num_nodes, parcel_weight, delta_T, delta_D)

        end_time = time.time()
        ep_time.append((end_time-start_time))

        print(f"\n\nEp {ep}: \t Instance: {testproblem} \t Time: {(end_time-start_time):.6f} seconds")
        print(f'\nSolution: \nTruck route: {truck_route}\nTruck time to node: {time_array}\nDrone routes: {drone_routes} \n\nTimespan: {timespan}')
        print(f'\n\nNumber of drone customer: {len(drone_routes)} \nNumber of truck customer: {len(truck_route)-2}')

        objective_values.append(timespan)

        if testproblem not in results:
            results[testproblem] = []

        results[testproblem].append({
            "test_instance": testproblem,
            "truck_cus": len(truck_route)-2,
            "drone_cus": len(drone_routes),
            "inference_time": round((end_time-start_time), 5),
            "timespan": round(timespan, 5)
        })

    print("============================================================================================")
    print("Avg computation time: ", np.mean(ep_time))
    print("Avg timespan: ", np.mean(objective_values))
    print("============================================================================================")

    # Plot
    plot_map(locations, truck_route, drone_routes)

    # Saving datas
    # Convert to DataFrame for CSV export
    if will_save:
        flattened_results = []
        for test_instance, values in results.items():
            flattened_results.extend(values)

            # Append an empty row as a separator
            flattened_results.append({"test_instance": "------",
                                    "truck_cus": "------", "drone_cus": "------", 
                                    "inference_time": "------", "timespan": "------"})


        df = pd.DataFrame(flattened_results)

        # Add a summary row
        summary_row = {
            "test_instance": "MEAN",
            "truck_cus": "",
            "drone_cus": "",
            "inference_time": f"{np.mean(ep_time):.4f}",
            "timespan": f"{np.mean(objective_values):.4f}",
        }

        df = pd.concat([df, pd.DataFrame([summary_row])], ignore_index=True)

        # Save to CSV
        csv_filename = f"result/{args.algorithm}/{city_name}_{num_nodes}nodes.csv"
        os.makedirs(os.path.dirname(csv_filename), exist_ok=True)
        df.to_csv(csv_filename, index=False)

        print('Saved the testing result to ', csv_filename)
