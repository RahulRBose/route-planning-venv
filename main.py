from math import radians, cos, sin, asin, sqrt

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

class Location:
    def __init__(self, name, lat, lng, time_window = None) -> None:
        self.lat = lat
        self.lng = lng
        self.name = name
        self.time_window = time_window
    def __str__(self):
        return self.name
        # return "{} - {} - {} - {}".format(self.name, self.lat, self.lng, self.time_window if self.time_window else "None")

def calculate_harvensine_distance(l1: Location, l2: Location) -> float:
    #convert degrees to radians
    lon1 = radians(l1.lng)
    lat1 = radians(l1.lat)
    lon2 = radians(l2.lng)
    lat2 = radians(l2.lat)

    #harvensine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 #radius of earth in km
    return c * r * 1000

def create_time_matrix_and_time_window(engineer_location: Location, sr_locations: list[Location]):
    '''
    time_matrix is a n*n matrix storing time difference between each pair of locations - including sr and engineer locations
    '''
    #here n = sr_count + 1 (1: engineer location)

    location_list = [engineer_location] + sr_locations
    location_count = len(location_list)

    time_window = list(map(lambda loc: loc.time_window, location_list))
    time_matrix = [[-1 for _ in range(location_count)] for _ in range(location_count)]

    for i in range(location_count):
        for j in range(location_count):
            loc_1 = location_list[i]
            loc_2 = location_list[j]
            if(loc_2.name == "engineer_start"): #if the destination is depot, consider the travel time as 0 to ignore this path
                time_matrix[i][j] = 0
                continue
            elif(i == j):
                time_matrix[i][j] = 0
            elif time_matrix[j][i] != -1:
                time_matrix[i][j] = time_matrix[j][i]
            else:
                distance = calculate_harvensine_distance(loc_1, loc_2)
                #assuming 20kmph speed
                time_matrix[i][j] = int(distance//333.33)

    return time_matrix, time_window

def create_data_model(engineer_location, sr_locations, no_of_engineers):
    '''Create the data model for the problem'''
    data = {}
    time_matrix, time_window = create_time_matrix_and_time_window(engineer_location, sr_locations)
    data["time_matrix"] = time_matrix
    data["time_windows"] = time_window
    data["no_of_engineers"] = no_of_engineers
    data["depot"] = 0 # this implies the index of the start location. In time matrix, 0th pos is the engineer_location

    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    # Display dropped nodes.
    dropped_nodes = "Dropped nodes:"
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += f" {manager.IndexToNode(node)}"
    print(dropped_nodes)
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    for vehicle_id in range(data["no_of_engineers"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{manager.IndexToNode(index)}@{data["time_windows"][manager.IndexToNode(index)]}"
                f" Time({solution.Min(time_var)},{solution.Max(time_var)})"
                " -> "
            )
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{manager.IndexToNode(index)}"
            f" Time({solution.Min(time_var)},{solution.Max(time_var)})\n"
        )
        plan_output += f"Time of the route: {solution.Min(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
    print(f"Total time of all routes: {total_time}min")


def main():
    #0 - 240 : 10 to 2
    #240 - 480 : 2 to 6
    sr_locations = [
        Location("hoskote_1", 13.072926, 77.787838, (0, 240)),
        Location("hoskote_2", 13.073216, 77.798853, (0, 240)),
        Location("hoskote_3", 13.082329, 77.790608, (241, 480)),
        Location("koramangala_1", 12.927923, 77.627106, (0, 240)),
        Location("koramangala_2", 12.942789, 77.627859, (241, 480)),
        Location("koramangala_3", 12.927087, 77.606964, (241, 480)),
        Location("hsr_1", 12.911884, 77.637080, (241, 480)),
        Location("hsr_2", 12.907450, 77.647128, (0, 240)),
        Location("hsr_3", 12.900354, 77.642374, (0, 240)),
        Location("jp_1", 12.913554, 77.592597, (241, 480)),
        Location("jp_2", 12.932962, 77.588990, (0, 240)),
        Location("jp_3", 13.910040, 77.576794, (241, 480)),
    ]

    #lets assume all engineers start from koramangala - 4th block

    engineer_location = Location("engineer_start", 12.937400, 77.613243, (0,480))
    no_of_engineers = 6

    '''
    ALGO START
    '''
    
    data = create_data_model(engineer_location, sr_locations, no_of_engineers)

    for row in data["time_matrix"]:
        print(row)

    print("\n", data["time_windows"])

    #ALGO LOGIC

    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["no_of_engineers"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # for vehicle_id in range(data["no_of_engineers"]):
    #     routing.SetFixedCostOfVehicle(240, vehicle_id)

    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        60,  # allow waiting time
        480,  # maximum time per vehicle
        True,  # Force start cumul to zero.
        time,
    )

    time_dimension = routing.GetDimensionOrDie(time)

    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    
    depot_idx = data["depot"]
    for vehicle_id in range(data["no_of_engineers"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
        )
    
    for i in range(data["no_of_engineers"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Allow to drop nodes.
    penalty = 1000
    for node in range(1, len(data["time_matrix"])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 5
    # search_parameters.log_search = True

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("NO SOLUTION")

if __name__ == "__main__":
    main()




