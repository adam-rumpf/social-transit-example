/**
Constraint function calculation (which, in this modified program, is actually treated as the objective).

Includes a variety of functions for calculating the constraint function values for a given solution. This requires calling the assignment model.
*/

#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "DEFINITIONS.hpp"
#include "network.hpp"
#include "assignment.hpp"

using namespace std;

extern string FILE_BASE;

/**
Constraint function class.

A variety of local attributes are used to store information required for calculating the constraint functions

Methods are used to execute different steps of the constraint function calculation process, which in turn requires the use of the assignment model.

NOTE: Currently leaving out the operator cost function, since it is irrelevant to our model.
*/
struct Constraint
{
	// Public attributes
	Network * Net; // pointer to the main transit network object
	NonlinearAssignment * Assignment; // pointer to the assignment model object
	pair<vector<double>, double> sol_pair; // flow vector/waiting time pair produced by assignment model
	double riding_weight; // user cost weight for in-vehicle travel time
	double walking_weight; // user cost weight for walking time
	double waiting_weight; // user cost weight for waiting time
	int stop_size; // number of stop nodes (also number of O/D nodes)

	// Public methods
	Constraint(Network *); // constructor that reads the operator cost, user cost, initial flow, and assignment model data and sets the network object pointer
	~Constraint(); // destructor deletes the assignment model object
	double calculate(const vector<int> &); // evaluates constraint functions for a given solution
	vector<double> user_cost_components(); // uses flow vector and waiting time scalar to calculate user cost components
};
