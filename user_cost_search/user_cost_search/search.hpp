/**
Main driver class of the TS/SA solution algorithm along with various logger classes.

Called by the main() function after all subroutine objects have been initialized, and uses them to conduct the search process.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "DEFINITIONS.hpp"
#include "constraints.hpp"
#include "network.hpp"

using namespace std;

extern string FILE_BASE;

typedef pair<pair<pair<int, int>, double>, pair<pair<int, int>, double>> neighbor_pair; // structure of neighborhood search output
typedef priority_queue<tuple<double, pair<int, int>, bool>, vector<tuple<double, pair<int, int>, bool>>, greater<tuple<double, pair<int, int>, bool>>> candidate_queue; // min-priority queue for storing objective/move/new tuples in the neighborhood search
typedef priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<pair<double, pair<int, int>>>> neighbor_queue; // min-priority queue for storing objective/move pairs at the end of the neighborhood search

// Global function prototypes
string vec2str(const vector<int> &); // returns string version of integer vector
vector<int> str2vec(string); // returns an integer vector for a given solution string

/**
Search object.

Contains search-related attributes and pointers to the main subroutine objects, and carries out the main solution algorithm.
*/
struct Search
{
	// Public attributes (object pointers)
	Network * Net; // pointer to main network object
	Constraint * Con; // pointer to main constraint object

	// Public attributes (search parameters and technical)
	int sol_size; // size of solution vector
	int step = 1; // step size for moves
	vector<int> line_min; // lower vehicle bounds for all lines
	vector<int> line_max; // upper vehicle bounds for all lines
	vector<int> max_vehicles; // maximum number of each vehicle type
	vector<int> vehicle_type; // vector of vehicle types for each line

	// Public attributes (solution algorithm memory)
	vector<int> sol_current; // current solution vector
	vector<int> sol_best; // best known solution vector
	double obj_current; // current objective value
	double obj_best; // best known objective value
	vector<int> current_vehicles; // number of each vehicle type currently in use
	int exhaustive_iteration; // iteration of exhaustive local search

	// Public methods
	Search(); // constructor initializes network, objective, constraint, and various logger objects
	~Search(); // destructor deletes network, objective, and constraint objects
	void solve(); // main driver of the solution algorithm
	vector<int> make_move(int, int); // returns the results of applying a move to the current solution
	void vehicle_totals(); // calculates total vehicles of each type in use
	void save_data(); // writes all current progress to the log files
	pair<pair<int, int>, double> best_neighbor(); // finds the best move from the current solution via exhaustive neighborhood search
	void exhaustive_search(); // conducts an exhaustive local search from the current solution
};
