# We publish the code under MIT license as it is the most permissible license we have managed to find and because Free Software Foundation **does not recommend** using informal licenses like "Do whatever you want" (https://www.gnu.org/licenses/license-list.en.html#informal).
#
# However, our knowledge in the area of licensing is limited, therefore feel free to contact the authors if you feel that this license does not work.
#
# Copyright 2020 https://github.com/undefiened/
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


import math
import simplejson
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from prepare_data import prepare_data


DRONES_CAPACITY = 100 # Capacity of a single drone
MAX_POINT_DEMAND = 20 # Maximum demand which a point can have. If a point has a higher demand, it will be splitted.
STOP_TIME_MIN = 15 # Time serving a single node takes in minutes
OPERATING_DAY_LENGTH_HOURS = 12 # Length of one operational day in hours.


USE_CACHE = False # If set to True then distance matrix computation will be cached. Useful if many computations for the same geographical area will be made.
HEURISTIC_TIME_LIMIT = 4800 # How much time in seconds the solver will be trying to solve the CVRP. After this time the best solution found will be returned as the result.

MAX_NUMBER_OF_STOPS = math.ceil((60/STOP_TIME_MIN)*OPERATING_DAY_LENGTH_HOURS) # The maximum number of stops a drone can make during a day
NUM_VEHICLES = math.ceil(200000/DRONES_CAPACITY) # we solve the problem for more tours to ensure that there is enough capacity to cover all population


def create_data_model(max_point_demand, use_cache=True):
    """
    Creates data model which will be feeded into Google OR-tools solver.

    :param max_point_demand: The maximum demand which a point can have.
    If a point has a higher demand, it will be splitted. Demand of a point must not exceed the capacity of a single drone!
    It is a Google OR-tools solver limitation.
    :param use_cache: If True then the computed distances matrix will be saved on the hard drive and reused later. Default: True
    :return: A data model.
    """
    points_data = prepare_data(max_point_demand, use_cache=use_cache)
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


def print_and_save_solution(data, manager, routing, assignment):
    """Prints the solution to the console and saves it to a json file."""
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
    with open('./results/capacity_{}_{}.json'.format(DRONES_CAPACITY, MAX_POINT_DEMAND), 'w') as f:
        simplejson.dump({
            'routes': routes,
            'total_load': total_load,
            'total_distance': total_distance,
            'number_of_drones_used': len(routes),
            'total_numer_of_stops': sum([x['number_of_stops'] for x in routes])
        }, f)


def main():
    """Solve the CVRP problem."""

    print('Drones capacity = {}'.format(DRONES_CAPACITY))

    # Instantiate the data of the problem
    data = create_data_model(MAX_POINT_DEMAND, USE_CACHE)

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Defining weights of the edges
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Addding capacity constraints.
    def demand_callback(from_index):
        """Returns the demand for tests of the node."""
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)

    def counter_callback(from_index):
        """Returns the number of stops done at the node."""
        from_node = manager.IndexToNode(from_index)
        return data['counter'][from_node]

    counter_callback_index = routing.RegisterUnaryTransitCallback(
        counter_callback)

    # Limiting the number of tests each drone can carry
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Limiting the overall number of nodes a drone can serve in one tour
    routing.AddDimensionWithVehicleCapacity(
        counter_callback_index,
        0,  # null capacity slack
        data['vehicle_max_number_of_stops'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Counter')

    # Setting parameters of the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = HEURISTIC_TIME_LIMIT
    search_parameters.log_search = True


    print('START SOLVING')
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        print_and_save_solution(data, manager, routing, assignment)


if __name__ == '__main__':
    main()
# [END program]