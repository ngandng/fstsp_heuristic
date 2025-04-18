
import numpy as np
import os
import pandas as pd

import networkx as nx
import osmnx as ox

from algorithms.fstsp_heuristic import truck_velocity, drone_velocity

def process_location_data(test_instance=None, locations=None, truck_distance=None, parcel_w=None):
        """ Return      locations
                        num_nodes
                        delivery network E_T, E_D
                        travel time matrix delta_T, delta_D

            Input       locations, real_distance_matrix, parcel_weight_matrix
                        OR test_instance name
        """

        if test_instance is not None:

            # Define folder paths
            problems_folder = f"{test_instance}/tbl_locations.csv"
            output_distance = f"{test_instance}/tbl_truck_travel_data_PG.csv"

            # Ensure the output directory exists
            os.makedirs(os.path.dirname(output_distance), exist_ok=True)

            # Read data from the input file
            # Define the column names
            column_names = ['nodeID', 'nodeType', 'latDeg', 'lonDeg', 'altMeters', 'parcelWtLbs']
            data = pd.read_csv(problems_folder, header=None, names=column_names, skiprows=1)

            # Extract locations and parcel weights
            locations = data[['latDeg', 'lonDeg']].values
            parcel_weight = data['parcelWtLbs'].values

            # Check if the distance file exists
            if os.path.exists(output_distance):
                # Load CSV file
                df = pd.read_csv(output_distance, header=None, names=["from", "to", "time", "distance"], skiprows=1)
                df.columns = df.columns.str.strip()

                # Get the number of unique locations
                unique_locations = sorted(df["from"].unique())
                n = len(unique_locations)

                # Initialize travel time matrix
                time_matrix = np.zeros((n, n))

                # Populate the matrix with travel times
                for _, row in df.iterrows():
                    i, j = int(row["from"]), int(row["to"])
                    time_matrix[i, j] = row["distance"]/(truck_velocity)

            else:   # if not available time matrix, calculating the time matrix and save

                _, real_distance_matrix = real_travel_distance(points=locations)
                time_matrix = real_distance_matrix / truck_velocity
                time_matrix = time_matrix  
                # Save in the same format as CSV
                time_data = []
                for i in range(len(locations)):
                    for j in range(len(locations)):
                        if i != j:
                            time_data.append([i, j, time_matrix[i, j], real_distance_matrix[i, j]])

                df_time = pd.DataFrame(time_data, columns=["from", "to", "time", "distance"])
                df_time.to_csv(output_distance, index=False)


        num_nodes = len(locations)

        euclidean_distance_matrix = np.zeros((num_nodes, num_nodes))

        # Calculate great-circle distance between each pair of points
        for i in range(num_nodes):
            for j in range(i+1, num_nodes):
                start_point = locations[i]
                end_point = locations[j]
                
                # Compute great-circle distance in kilometers
                distance = ox.distance.great_circle(start_point[0], start_point[1], end_point[0], end_point[1]) # in meters
                euclidean_distance_matrix[i, j] = distance
                euclidean_distance_matrix[j, i] = distance  # Symmetric matrix

        # Compute travel time matrices
        delta_T = time_matrix
        delta_D = euclidean_distance_matrix / drone_velocity
        
        # Checking travel matrix
        for i in range(len(delta_T)):
            for j in range(i+1, len(delta_T)):
                if delta_D[i,j] > delta_T[i,j]:
                    print("Check travel time matrix at node({},{}): Drone time = {}, Truck time = {}".format(i,j,delta_D[i,j],delta_T[i,j]))
            

        return locations, num_nodes, parcel_weight, delta_T, delta_D



def point_to_node_distance(lat, lon, node, G):
    """Compute the Euclidean distance from a point to the nearest graph node (in km)."""
    node_data = G.nodes[node]
    node_lat, node_lon = node_data['y'], node_data['x']
    return ox.distance.great_circle(lat, lon, node_lat, node_lon)



def real_travel_distance(points, G = None):
    num_point = len(points)
    distance_matrix = np.zeros((num_point, num_point))
    route_matrix = np.full((num_point, num_point), None)

    # Compute a bounding box covering all points
    latitudes, longitudes = zip(*points)
    center_lat, center_lon = np.mean(latitudes), np.mean(longitudes)

    # Define a sufficiently large area (bounding box size)
    max_dist = 15000

    if G is None:
        G = ox.graph_from_point((center_lat, center_lon), dist=max_dist, 
                                simplify=False,
                                network_type="drive")
        G = nx.MultiDiGraph(G)

    for i in range(num_point):
        for j in range(i+1, num_point):
            start_point, end_point = points[i], points[j]

            start_node = ox.distance.nearest_nodes(G, X=start_point[1], Y=start_point[0])   # X = lon, Y = lat
            end_node = ox.distance.nearest_nodes(G, X=end_point[1], Y=end_point[0])

            # If either node is missing, assign large penalty
            if start_node is None or end_node is None:
                distance_matrix[i, j] = distance_matrix[j, i] = float('inf')
                continue

            # Compute shortest path with fallback expansion
            shortest_route_distance = None
            # try to change the graph center first, before increasing the range
            temp_max_dist = max_dist
            while temp_max_dist <= 50000:   # maximum range is 100 kilometers
                try:
                    shortest_route_distance = nx.shortest_path_length(G, source=start_node, target=end_node, weight="length")
                    shortest_path = nx.shortest_path(G, source=start_node, target=end_node, weight="length")
                    break  # Exit loop if path is found
                except nx.NetworkXNoPath:
                    temp_max_dist += 5000  # Expand search radius
                    print(f"No path found between {start_point} and {end_point}, increasing radius to {temp_max_dist/1000}(km).")

                    G = ox.graph_from_point((start_point[0], start_point[1]), dist=temp_max_dist, network_type="drive")
                    G = nx.MultiDiGraph(G)

                    # update start node and end node
                    start_node = ox.distance.nearest_nodes(G, X=start_point[1], Y=start_point[0])   # X = lon, Y = lat
                    end_node = ox.distance.nearest_nodes(G, X=end_point[1], Y=end_point[0])

                except Exception as e:
                    print(f"Unexpected error: {e}")
                    break

            if shortest_route_distance is None:
                print(f"Failed to find a path between {start_point} and {end_point} after max expansion.")
                shortest_route_distance = float('inf')  # Assign large penalty

            # Compute additional distances to the nearest node
            additional_distance = (
                point_to_node_distance(*start_point, start_node, G) +
                point_to_node_distance(*end_point, end_node, G)
            )

            # Store total distance in meters
            total_distance = shortest_route_distance + additional_distance
            distance_matrix[i, j] = distance_matrix[j, i] = total_distance
            route_matrix[i, j] = shortest_path
            route_matrix[j, i] = list(reversed(shortest_path))  # Since it's undirected

    return route_matrix, distance_matrix  # Convert meters to km


def calc_timespan():
    #TODO: Implement this function
    pass