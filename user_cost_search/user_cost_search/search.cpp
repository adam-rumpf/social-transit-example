/// Search class methods.

#include "search.hpp"

/// Search constructor initializes Network, Objective, and Constraint objects and loads search parameters.
Search::Search()
{
	Net = new Network(); // network object
	Con = new Constraint(Net); // constraint function object
	sol_size = Net->lines.size(); // get solution vector size
	srand(time(NULL)); // seed random number generator
}

/// Search destructor deletes Network, Objective, and Constraint objects created by the constructor.
Search::~Search()
{
	delete Net;
	delete Con;
}

/// Main driver of the solution algorithm. Calls main search loop and handles final output.
void Search::solve()
{
	// Initialize neighborhood search solution container and set references to its elements
	neighbor_pair nbhd_sol;
	pair<int, int> & nbhd_sol1 = nbhd_sol.first.first;
	pair<int, int> & nbhd_sol2 = nbhd_sol.second.first;
	double & nbhd_obj1 = nbhd_sol.first.second;
	double & nbhd_obj2 = nbhd_sol.second.second;

	// Determine total vehicle bounds
	max_vehicles.resize(Net->vehicles.size());
	for (int i = 0; i < Net->vehicles.size(); i++)
		max_vehicles[i] = Net->vehicles[i]->max_fleet;

	// Determine line fleet bounds
	line_min.resize(sol_size);
	line_max.resize(sol_size);
	for (int i = 0; i < sol_size; i++)
	{
		line_min[i] = Net->lines[i]->min_fleet;
		line_max[i] = Net->lines[i]->max_fleet;
	}

	// Determine current total vehicle usage and establish vehicle type vector
	vehicle_type.resize(Net->lines.size());
	for (int i = 0; i < Net->lines.size(); i++)
		vehicle_type[i] = Net->lines[i]->vehicle_id;
	vehicle_totals();

	// Handle exhaustive search

	exhaustive_iteration = 0;

	cout << "\n============================================================" << endl;
	cout << "Final exhaustive search" << endl;
	cout << "============================================================" << endl << endl;

	// Set current solution to best
	sol_current = sol_best;
	obj_current = obj_best;
	vehicle_totals();

	// Initialize local search method
	exhaustive_search();

	// Perform final saves after search completes
	save_data();
}

/**
Generates the solution vector resulting from a specified move.

Requires an ADD line ID and a DROP line ID. Use an ID of "NO_ID" to ignore one of the move types.

Returns a vector resulting from applying the specified move to the current solution.
*/
vector<int> Search::make_move(int add_id, int drop_id)
{
	vector<int> sol = sol_current;

	// ADD move
	if (add_id != NO_ID)
		sol[add_id] += step;

	// DROP move
	if (drop_id != NO_ID)
		sol[drop_id] -= step;

	return sol;
}

/// Calculates total number of each vehicle type in use for the current solution, and updates vehicle total variable.
void Search::vehicle_totals()
{
	current_vehicles = vector<int>(Net->vehicles.size(), 0);
	for (int i = 0; i < Net->lines.size(); i++)
		current_vehicles[vehicle_type[i]] += sol_current[i];
}

/// Writes current memory structures to the output logs.
void Search::save_data()
{
	MemLog->save_memory();
	SolLog->save_solution();
}

/**
Finds the absolute best neighbor of the current solution.

Returns a move/objective value pair corresponding to the best neighbor. If no neighbor has an objective value strictly lower than the given solution (meaning that the given solution is locally optimal), the returned solution will consist of the NO_ID move pair and an infinite objective.

This is for use in an exhaustive local search. Every possible ADD and DROP move from the given solution is considered (we do not consider SWAP moves since there are so many). Tabu rules are ignored but all other constraints are enforced.
*/
pair<pair<int, int>, double> Search::best_neighbor()
{
	// Current best known neighbor objective and move
	pair<int, int> top_move = make_pair(NO_ID, NO_ID);
	double top_objective = INFINITY;

	// Consider every possible ADD move
	for (int choice = 0; choice < sol_size; choice++)
	{
		// Filter out moves that would violate a line fleet bound
		if (sol_current[choice] + step > line_max[choice])
			// Skip ADD moves that would exceed a line's vehicle bound
			continue;
		if (current_vehicles[vehicle_type[choice]] + 1 > max_vehicles[vehicle_type[choice]])
			// Skip ADD moves that would exceed a total vehicle bound
			continue;

		// Initialize candidate solution containers
		vector<int> sol_candidate = make_move(choice, NO_ID); // solution vector resulting from chosen ADD
		double obj_candidate; // objective of candidate solution
		int feas; // candidate solution feasibility status

		// Calculate its objective and create a tentative log entry
		feas = FEAS_UNKNOWN;
		clock_t start = clock(); // objective calculation timer
		obj_candidate = Obj->calculate(sol_candidate); // calculate objective value
		double candidate_time = (1.0*clock() - start) / CLOCKS_PER_SEC; // objective calculation time

		// Filter out moves that do not improve on the current solution or best known neighbor
		if ((obj_candidate >= obj_current) || (obj_candidate >= top_objective))
			continue;

		// Evaluate the feasibility of the candidate solution
		if (feas == FEAS_UNKNOWN)
		{
			// If feasibility is unknown, calculate its constraints and create a full log entry
			clock_t start = clock(); // constraint calculation timer
			pair<int, vector<double>> con_candidate = Con->calculate(sol_candidate); // calculate feasibility status and constraint vector
			double candidate_time = (1.0*clock() - start) / CLOCKS_PER_SEC; // constraint calculation time
			if (con_candidate.first == FEAS_FALSE)
				// Skip candidate if discovered to be infeasible
				continue;
		}

		// If we've made it this far, the candidate should be kept
		top_move = make_pair(choice, NO_ID);
		top_objective = obj_candidate;
	}
	cout << '.';

	// Consider every possible DROP move
	for (int choice = 0; choice < sol_size; choice++)
	{
		// Filter out moves that would violate a line fleet bound
		if (sol_current[choice] - step < line_min[choice])
			// Skip DROP moves that would fall below a line's vehicle bound
			continue;
		if (current_vehicles[vehicle_type[choice]] - 1 < 0)
			// Skip DROP moves that would result in negative vehicles
			continue;

		// Initialize candidate solution containers
		vector<int> sol_candidate = make_move(NO_ID, choice); // solution vector resulting from chosen DROP
		double obj_candidate; // objective of candidate solution
		int feas; // candidate solution feasibility status

		// Calculate its objective and create a tentative log entry
		feas = FEAS_UNKNOWN;
		clock_t start = clock(); // objective calculation timer
		obj_candidate = Obj->calculate(sol_candidate); // calculate objective value
		double candidate_time = (1.0*clock() - start) / CLOCKS_PER_SEC; // objective calculation time

		// Filter out moves that do not improve on the current solution or best known neighbor
		if ((obj_candidate >= obj_current) || (obj_candidate >= top_objective))
			continue;

		// Evaluate the feasibility of the candidate solution
		if (feas == FEAS_UNKNOWN)
		{
			// If feasibility is unknown, calculate its constraints and create a full log entry
			clock_t start = clock(); // constraint calculation timer
			pair<int, vector<double>> con_candidate = Con->calculate(sol_candidate); // calculate feasibility status and constraint vector
			double candidate_time = (1.0*clock() - start) / CLOCKS_PER_SEC; // constraint calculation time
			if (con_candidate.first == FEAS_FALSE)
				// Skip candidate if discovered to be infeasible
				continue;
		}

		// If we've made it this far, the candidate should be kept
		top_move = make_pair(NO_ID, choice);
		top_objective = obj_candidate;
	}
	cout << '.';

	// Return the best solution vector
	cout << endl;
	return make_pair(top_move, top_objective);
}

/**
Conducts an exhaustive, greedy local search from the current solution.

Each iteration of the search moves to the neighbor with the best objective value. The search ends when local optimality is achieved.
*/
void Search::exhaustive_search()
{
	// Find best neighbor
	pair<pair<int, int>, double> move = best_neighbor();

	// Continue main loop until reaching local optimality
	while (move.second < INFINITY)
	{
		clock_t start = clock(); // iteration timer

		exhaustive_iteration++;
		cout << "\n---------- Exhaustive Search Iteration " << exhaustive_iteration << " ----------\n" << endl;

		// Make local move and update objective and vehicle usage
		sol_current = make_move(move.first.first, move.first.second);
		obj_current = move.second;
		vehicle_totals();

		// Repeat neighborhood search
		move = best_neighbor();
	}
}
