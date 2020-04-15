# Computing the number of drones needed for delivery of tests

## Description
This repository contains code used for computing drones tours in our research work on delivery of tests.

## Requirements
* Python 3+
* Google OR-tools (https://developers.google.com/optimization)
* Python packages:
    * simplejson (https://github.com/simplejson/simplejson)
    * geopy (https://github.com/geopy/geopy)
    * numpy (https://numpy.org/)

Optional:
* Gurobi (https://www.gurobi.com/)
* gurobipy package for Python (https://www.gurobi.com/)

## Usage
We split the problem into two separate subproblems:
* Finding the best tours to cover the whole population.
* Assigning tours to drones to minimize the maximum makespan.

We note that the tours obtained by running Google OR-tools are **not** optimal as they are computed by a heuristic as CVRP is NP-hard problem and its solving to optimality requires extreme computational resources.

### Preparing data
For our computations we used geographical data stored as a list of points with "Population" property in "geojson" file (we obtained it by saving a Point Feature in QGIS software - https://qgis.org/en/site/).

One possibility is to simply make changes in `prepare_data.py` file to make it work with your data.
Or, a json file in the following format can be used (please note that we describing here only necessary fields and structure of the file, other fields are allowed):

    {
    "features": [
    { "properties": { "TotBef": 111 }, "geometry": { "coordinates": [ 16.080061264303954, 58.554477805317212 ] } },
    { "properties": { "TotBef": 222 }, "geometry": { "coordinates": [ 16.090061264303954, 58.554477805317212 ] } },
    ]
    }

Where `TotBef` property represents the population living in a cell centered in coordinate `geometry` -> `coordinates`.

Also please see `centroids100x100.geojson` as an example.

### Computing tours
Running `compute_tours.py` will compute tours. There are several parameters in the top of this file which can be adjusted to select drones capacity, etc -- they all are explained in the code file itself with comments.

### Assigning tours to drones
Running `assign_tours.py` will compute the optimal assignment of tours to drones which minimizes the maximum makespan. Again, there are several parameters in the top of the file, which are explained in the code comments.

## License
The data in the file `centroids100x100.geojson` is obtained from the geographical data courtesy of Statistics Sweden (https://scb.se/) provided by Swedish University of Agricultural Sciences (https://www.slu.se/) under FUK (Forskning, utbildning och kulturverksamhet) license (https://www.geodata.se/anvanda/forskning-utbildning-och-kulturverksamheter/).

We publish the code under MIT license as it is the most permissible license we have managed to find and because Free Software Foundation **does not recommend** using informal licenses like "Do whatever you want" (https://www.gnu.org/licenses/license-list.en.html#informal).

However, our knowledge in the area of licensing is limited, therefore feel free to contact the authors if you feel that this license does not work.

Copyright 2020 https://github.com/undefiened/

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
