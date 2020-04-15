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


import pickle
from copy import deepcopy
from os import path

from geopy.distance import distance
import math
import numpy as np
import simplejson


HOSPITAL = {'lon': 16.1788, 'lat': 58.5633, 'Population': 0} # Location of the test distribution center


def near_split(x, num_bins):
    """
    Splits an integer number "x" into a list of "num_bins" bins while keeping the difference between values as small as possible.

    E.g., for x = 50, num_bins = 3 the result will be [17, 17, 16]

    A courtesy of https://stackoverflow.com/a/48918717

    :param x: Integer number representing the number to divide into bins
    :param num_bins: Integer number representing the number of bins
    :return: A list of integer numbers.
    """
    quotient, remainder = divmod(x, num_bins)
    res = [quotient + 1] * remainder + [quotient] * (num_bins - remainder)
    return res


def load_data():
    """
    Loads the data from geojson file obtained from QGIS. The data consists of points with associated population values.
    :return:
    """
    data = simplejson.load(open('centroids100x100.geojson', 'r'))
    all_points = [HOSPITAL]

    for feature in data['features']:
        if feature['properties']['TotBef'] > 0:
            all_points.append({
                'lon': feature['geometry']['coordinates'][0],
                'lat': feature['geometry']['coordinates'][1],
                'Population': feature['properties']['TotBef']
            })

    return all_points


def compute_distance_matrix(all_points):
    """
    Computes a distance matrix between all pairs of points
    :param all_points: A list of points
    :return: A matrix where every entry represents the distance between corresponding points
    """
    distance_matrix = np.zeros((len(all_points), len(all_points)))

    for x in range(len(all_points)):
        print(x)
        for y in range(x + 1, len(all_points)):
            p1 = all_points[x]
            p2 = all_points[y]
            if abs(p1['lat'] - p2['lat']) > 0 or abs(p1['lon'] - p2['lon']) > 0:
                dist = distance((p1['lat'], p1['lon']), (p2['lat'], p2['lon'])).m
            else:
                dist = 0

            distance_matrix[x, y] = dist
            distance_matrix[y, x] = dist

    return distance_matrix


def split_dense_points(orig_points, orig_distance_matrix, max_point_capacity):
    """
    This function splits points which population exceeds "max_point_capacity" into several points with smaller capacity
    and the same coordinates.
    It is used because Google OR-tools does not work with the case, when a vehicle has to serve the same point twice,
    so problem instances where some point has demand exceeding vehicles capacity are becoming infeasible.

    :param orig_points: A list of points
    :param orig_distance_matrix: A distance matrix
    :param max_point_capacity: Maximum capacity which can be assigned to a point.
    :return: A new list of points and a new distance matrix
    """
    points = []
    indices_of_orig_points = []

    for i in range(len(orig_points)):
        point = orig_points[i]

        if point['Population'] > 0:
            num_bins = math.ceil(point['Population'] / max_point_capacity)
            for population in near_split(point['Population'], num_bins):
                new_point = deepcopy(point)
                new_point['Population'] = population
                points.append(new_point)
                indices_of_orig_points.append(i)
        else:
            points.append(point)
            indices_of_orig_points.append(i)

    distance_matrix = np.zeros((len(points), len(points)))

    for x in range(distance_matrix.shape[0]):
        for y in range(distance_matrix.shape[1]):
            distance_matrix[x][y] = orig_distance_matrix[indices_of_orig_points[x]][indices_of_orig_points[y]]

    return points, distance_matrix


def create_data_model(max_point_capacity, use_cache=True):
    all_points = load_data()

    if use_cache:
        filename = 'results/distance_matrix.p'

        if not path.exists(filename):
            distance_matrix = compute_distance_matrix(all_points)
            with open(filename, 'wb') as f:
                pickle.dump(distance_matrix, f)
        else:
            with open(filename, 'rb') as f:
                distance_matrix = pickle.load(f)
    else:
        distance_matrix = compute_distance_matrix(all_points)

    splitted_points, splitted_distance_matrix = split_dense_points(all_points, distance_matrix, max_point_capacity)

    data = {
        'distance_matrix': splitted_distance_matrix,
        'all_points': splitted_points
    }

    return data


def prepare_data(max_point_capacity, use_cache=True):
    res = create_data_model(max_point_capacity, use_cache=use_cache)
    return res
