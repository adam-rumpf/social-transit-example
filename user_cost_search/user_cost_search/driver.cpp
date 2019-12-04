/**
The main function for the search algorithm, modified to run as a user cost-optimizing local search.

Responsible for reading input data, initializing objects, and finally calling the search function, which is where most of the algorithm is actually conducted.

The exit code should correspond to the circumstances of the exit.
*/

#include <csignal>
#include <iostream>
#include "DEFINITIONS.hpp"
#include "search.hpp"

using namespace std;

// Global search object pointer
Search * Solver;

// Global file base name
string FILE_BASE = "";

/// Main driver
int main()
{
	// Initialize search object
	Solver = new Search();

	// Call main solver
	Solver->solve();

	// Delete solver to automate shutdown process
	delete Solver;

	cin.get();
	return SUCCESSFUL_EXIT;
}
