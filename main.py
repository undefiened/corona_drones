import pickle
from os import path

import math
import simplejson
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from prepare_data import prepare_data


DRONES_CAPACITY = 10
MAX_POINT_CAPACITY = 10
STOP_TIME_MIN = 15
OPERATING_DAY_LENGTH_HOURS = 12


USE_CACHE = True
HEURISTIC_TIME_LIMIT = 4800
# HEURISTIC_TIME_LIMIT = 120

MAX_NUMBER_OF_STOPS = math.ceil((60/STOP_TIME_MIN)*OPERATING_DAY_LENGTH_HOURS)
NUM_VEHICLES = math.ceil(200000/DRONES_CAPACITY) # we solve the problem for more tours to ensure that there is enough capacity to cover all population

def create_data_model(max_point_capacity, use_cache=True):
    points_data = prepare_data(max_point_capacity, use_cache=use_cache)
    all_points = points_data['all_points']
    distance_matrix = points_data['distance_matrix']

    number_of_vehicles = max(math.ceil(len(all_points)/MAX_NUMBER_OF_STOPS) + 20, NUM_VEHICLES)
    data = {
        'all_points': all_points,
        'distance_matrix': distance_matrix,
        'demands': [x['Population'] for x in all_points],
        'counter': [1 if x['Population'] > 0 else 0 for x in all_points],
        'vehicle_capacities': [DRONES_CAPACITY for _ in range(number_of_vehicles)],
        'vehicle_max_number_of_stops': [MAX_NUMBER_OF_STOPS for _ in range(number_of_vehicles)],
        'num_vehicles': number_of_vehicles,
        'depot': 0
    }

    print('Total demand = {}'.format(sum(data['demands'])))

    return data



# [START solution_printer]
def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_distance = 0
    total_load = 0
    routes = []
    for vehicle_id in range(data['num_vehicles']):
        route = []
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            point = data['all_points'][node_index]
            point['load'] = route_load
            route.append(point)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        if route_load > 0:
            routes.append({
                'stops': route,
                'distance': route_distance,
                'load': route_load,
                'number_of_stops': len(route)
            })
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    with open('./results/capacity_{}_{}.json'.format(DRONES_CAPACITY, MAX_POINT_CAPACITY), 'w') as f:
        simplejson.dump({
            'routes': routes,
            'total_load': total_load,
            'total_distance': total_distance,
            'number_of_drones_used': len(routes),
            'total_numer_of_stops': sum([x['number_of_stops'] for x in routes])
        }, f)

    # [END solution_printer]


def main():
    """Solve the CVRP problem."""

    print('Drones capacity = {}'.format(DRONES_CAPACITY))

    # Instantiate the data problem.
    # [START data]
    data = create_data_model(MAX_POINT_CAPACITY, USE_CACHE)
    # [END data]

    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    # [END index_manager]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)

    # [END routing_model]

    # Create and register a transit callback.
    # [START transit_callback]
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # [END transit_callback]

    # Define cost of each arc.
    # [START arc_cost]
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # [END arc_cost]

    # Add Capacity constraint.
    # [START capacity_constraint]
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)

    def counter_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['counter'][from_node]

    counter_callback_index = routing.RegisterUnaryTransitCallback(
        counter_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    routing.AddDimensionWithVehicleCapacity(
        counter_callback_index,
        0,  # null capacity slack
        data['vehicle_max_number_of_stops'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Counter')
    # [END capacity_constraint]

    # Setting first solution heuristic.
    # [START parameters]
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = HEURISTIC_TIME_LIMIT
    search_parameters.log_search = True

    # [END parameters]

    # Solve the problem.
    # [START solve]
    print('START SOLVING')
    assignment = routing.SolveWithParameters(search_parameters)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    if assignment:
        print_solution(data, manager, routing, assignment)
    # [END print_solution]


if __name__ == '__main__':
    main()
# [END program]