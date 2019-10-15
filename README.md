# social-transit-example

A script to procedurally generate small-scale example networks for public transit design with social access objectives.

This is part of my research project related to public transit design with social access objectives, which involves a case study of the Chicago bus network and primary care access. The purpose of this program is to generate small-scale synthetic networks for use in testing the solution algorithm. It will also be useful for tuning the tabu search/simulated annealing hybrid algorithm's search parameters as well as conducting sensitivity analysis.

I would not expect this program to be of use to anyone outside of my research group, but it is being made available if anyone is interested.

## Summary

This program is meant to procedurally generate test instances which can be quickly evaluated by hand to decide whether they are "interesting". Calculating the operator and user costs of the network is expensive, so we will consider only the objective value as a function of the line fleet sizes. An "interesting" example should have a "rough" objective function which presents a challenge for the solution algorithm. A quick way to test this would be to choose a few random pairs of decision variables and vary each pair to generate tables of objective values. We would like for all of these tables to show nonconvex behavior.

The procedural generation should rely on a set of input parameters which can be quickly changed by the user. Based on these parameters, a random problem instance should be generated along with some metrics for evaluating whether the instance is interesting. There should also be some functions for quickly generating output files based on the current example network.

## Output Format

See `format.txt` for a full description of the output file format. In particular we are interested in outputting the following:

* `arc_data.txt`: arc ID, arc type, arc line ID, arc tail node ID, arc head node ID, arc cost
* `node_data.txt`: node ID, node name, node type, node line ID, node value (population or facilty weight)
* `objective_data.txt`: elements, number of population centers to include, FCA cutoff, gravity parameter
* `od_data.txt`: unique ID, origin node ID, destination node ID, travel volume
* `transit_data.txt`: line ID, line name, vehicle type, initial fleet size, total circuit time, scaling factor, fleet lower bound, fleet upper bound, boarding fare, initial line frequency, initial line capacity
* `vehicle_data.txt`: vehicle type ID, vehicle type name, vehicle type bound, vehicle type capacity, vehicle operating cost
