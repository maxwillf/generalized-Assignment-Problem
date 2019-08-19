#ifndef __GAP_SOLVER__
#define __GAP_SOLVER__

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <cctype>
#include "gapProblem.hpp"

#define INF ((unsigned) ~0)

enum class HeuristicPolicy {MAXCOST,MINRES};

class gapSolver
{

private:
  std::vector<gapProblem> problemSet;
  HeuristicPolicy policy;
  
  
  HeuristicPolicy stringToPolicy(std::string policyStr);

public:
  void readProblemSetFile(std::string path);

  gapSolver();
  gapSolver(std::string path, std::string Policy ="MAXCOST");

  void solve();
  int getMinimumResourceCostAgent(int job, gapProblem problem);
  int getMaximumCostAvailableAgent(int job, gapProblem problem);
};

#endif
