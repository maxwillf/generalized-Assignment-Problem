#include "gapSolver.hpp"

int main(int argc, char *argv[])
{
	
  
  gapSolver solver;
  std::string policy;
  
  if(argc != 2 && argc != 3) {
		std::cerr << "Please give a testCase file as input (and optionally, a policy)" << std::endl;
		exit(-1);
  }

  if(argc == 2){
  solver = gapSolver(argv[1]);
  }

  else if(argc == 3){
  policy = std::string(argv[2]);
  solver = gapSolver(argv[1],policy);
  }
 
  solver.solve();

	return 0;
}
