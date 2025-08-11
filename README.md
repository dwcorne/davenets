# davenets
DAVENETS is a simulator of timetabled transport of demand across network structures, initially developed for bus passenger/parcel system simulation

DAVENETS stands for Discrete And VErsatile NEtwork Transport Simulation.
It was initiated in 2025 as part of the TRANSIT EPRSC project (and inspired by the earlier SOAST project), in order to investigate passenger/parcel systems
operating on bus routes. 

A command line call of davenets looks like this:

your_prompt% davenets network_file.txt routes_file.txt demand_file.txt results.csv

The network file describes a network in terms of nodes and links. In this README we will describe only the most simple and efficient approach to defining the network. 
Given that the designer has one or more bus routes in mind, this simple approach is simply to identify each node with a bus stop (or terminus) and each link as an arc, 
with a given speed, between two stops. Each node (stop) also has a category (e.g. 'city' or 'rural') which is used when generating simulated demand.

The routes file describes the actual bus routes that operate on the network. A bus route has a name, and a sequence of stops. Once a route has been defined, 
zero or more individual buses can be defined which use this route. The individual buses will be defined by parameters (capacities for people and parcels), and
a list of times, precisely one per stop on the route.

Finally, the demand file describes demand in terms of a series of trips between pairs of stops.
Perusal of examples of these files should be enough to understand how they are constructed.
