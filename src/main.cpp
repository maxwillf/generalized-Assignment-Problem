#include "gapSolver.hpp"

int main(int argc, char *argv[])
{
	
	if(argc != 2) {
		std::cerr << "Please give a testCase file as input" << std::endl;
		exit(-1);
  }

  gapSolver solver(argv[1]);
  solver.solveFirstProblem();

	return 0;
}
