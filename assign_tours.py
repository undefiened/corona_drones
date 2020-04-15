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


import itertools
from typing import List
# import gurobipy as gp
from gurobipy import *
import numpy as np
import simplejson

IP_ABSOLUTE_GAP = 1000 # value in seconds of the absolute gap after achieving which IP solver will terminate
IP_TIME_LIMIT = 600 # value in seconds of the time limit after which IP solver will terminate

capacities = [(20, 10), (50, 20), (60, 20), (100, 20), (200, 20), (500, 20), (1000, 1000)] # a list of tuples of the format: (capacity, max point capacity)
speed_km_h = 60 # Speed of the drones in km/h
stop_time_min = 15 # Time drones spend in a cell


stop_time_sec = stop_time_min*60
speed_m_s = speed_km_h*1000/(60*60)


def run_IP(number_of_drones: int, jobs_durations: List[float]):
    """
    Solves Integer Program for Minimum Makespan Scheduling problem using Gurobi solver.
    It might be not necessary to solve this IP to optimality, therefore we leave IP_ABSOLUTE_GAP parameter which allows
    solver to stop when the best obtained solution is within IP_ABSOLUTE_GAP from optimality.

    Also, we make available an approximation LPT algorithm for this problem which finds solution within 4/3 - 1/3m from the optimum.
    :param number_of_drones: Between how many drones the tours will be divided. Should be a positive integer number.
    :param jobs_durations: A list of jobs durations.
    :return: A dictionary of assignments of drones to jobs (in the format "drone_id: [jobs_ids]") and a list of total jobs durations for every drone.
    """
    print(number_of_drones)
    m = Model("Minimum makespan scheduling")
    drones = range(number_of_drones)
    jobs = range(len(jobs_durations))
    jobs_to_drones_indices = list(itertools.product(drones, jobs))

    jobs_to_drones = m.addVars(jobs_to_drones_indices, name="drone_job", vtype=GRB.BINARY)
    max_time_length = m.addVar(name="max_time_length", vtype=GRB.CONTINUOUS)
    m.addConstrs((quicksum([jobs_to_drones[(drone, job)] for drone in drones]) == 1 for job in jobs)) #each job is assigned somewhere
    m.addConstrs((quicksum([jobs_to_drones[(drone, job)]*jobs_durations[job] for job in jobs]) <= max_time_length for drone in drones))

    m.setObjective(max_time_length, GRB.MINIMIZE)
    m.Params.MIPGapAbs = IP_ABSOLUTE_GAP
    m.Params.TimeLimit = IP_TIME_LIMIT
    m.optimize()

    jobs_assignment = {}

    for drone in drones:
        jobs_assignment[drone] = [job for job in jobs if jobs_to_drones[(drone, job)].X >= 0.8]

    bins = [sum([jobs_durations[job] for job in jobs if jobs_to_drones[(drone, job)].X >= 0.8]) for drone in drones]

    return jobs_assignment, bins


def lpt(number_of_drones: int, jobs: List[int]):
    """
    Does the same as run_IP, but instead of solving IP, it runs Longest Processing Time algorithm which gives
    a solution within 4/3 - 1/(3m) from the optimum, where m is the number of drones.
    :param number_of_drones: Between how many drones the tours will be divided. Should be a positive integer number.
    :param jobs_durations: A list of jobs durations.
    :return: A dictionary of assignments of drones to jobs (in the format "drone_id: [jobs_ids]") and a list of total jobs durations for every drone.
    """
    bins = np.zeros(number_of_drones)
    jobs_assignment = {}

    original_indices = np.argsort(jobs)[::-1]
    sorted_jobs = np.sort(jobs)[::-1]

    for i in range(len(original_indices)):
        job_length = sorted_jobs[i]
        job_original_index = original_indices[i]

        min_bin_index = np.argmin(bins)

        bins[min_bin_index] = bins[min_bin_index] + job_length

        if min_bin_index not in jobs_assignment:
            jobs_assignment[min_bin_index] = []

        jobs_assignment[min_bin_index].append(job_original_index)

    return jobs_assignment, bins.tolist()


def compute_jobs_durations(tours):
    """
    Computes the durations of every tour using drone speed and time it takes at every stop.
    :param tours: A dictionary in the format from compute_tours.py
    :return: A list of jobs durations
    """
    jobs = []

    for route in tours['routes']:
        number_of_stops = 0
        for stop_id in range(1, len(route['stops'])):
            previous_stop_id = stop_id - 1

            if not (route['stops'][previous_stop_id]['lat'] == route['stops'][stop_id]['lat']
                    and route['stops'][previous_stop_id]['lon'] == route['stops'][stop_id]['lon']):
                number_of_stops = number_of_stops + 1

        jobs.append((route['distance'] / speed_m_s) + number_of_stops * stop_time_sec)

    return jobs


def main():
    for capacity in capacities:
        with open('./results/capacity_{}_{}.json'.format(capacity[0], capacity[1]), 'r') as f:
            routes = simplejson.load(f)

            jobs = compute_jobs_durations(routes)

            results = []

            for number_of_drones in range(1, 100):
                # jobs_assignment, bins = lpt(number_of_drones, jobs) # use this line instead of the next if LPT algorithm is preferred
                jobs_assignment, bins = run_IP(number_of_drones, jobs)
                results.append({
                    'number_of_drones': number_of_drones,
                    'jobs_assignment': [[int(x) for x in value] for (key, value) in jobs_assignment.items()],
                    'bins': bins,
                    'routes': [[routes['routes'][route] for route in jobs] for (drone_id, jobs) in jobs_assignment.items()]
                })

            with open('./results/schedule_ip_capacity_{}_{}'.format(capacity[0], capacity[1]), 'w') as res_f:
                simplejson.dump({'assignments': results, 'all_routes': routes}, res_f)


if __name__ == '__main__':
    main()
