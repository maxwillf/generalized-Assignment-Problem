#ifndef __GAP_SOLVER__
#define __GAP_SOLVER__

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <cctype>
#include <set>
#include "gapProblem.hpp"

#define INF ((unsigned) ~0)

enum class HeuristicPolicy {MAXCOST,MINRES};

class gapSolver
{

private:
  std::vector<gapProblem> problemSet;
  std::set<std::string> visitedNodesSet;
  HeuristicPolicy policy;
  
  
  HeuristicPolicy stringToPolicy(std::string policyStr);

public:
  void readProblemSetFile(std::string path);

  gapSolver();
  gapSolver(std::string path, std::string Policy ="MAXCOST");

  gapProblem heuristicSolve(gapProblem problem);
  gapProblem branchAndBound(gapProblem problem);
  void heuristicSolveAll();

  int getMinimumResourceCostAgent(int job, gapProblem problem);
  int getMaximumCostAvailableAgent(int job, gapProblem problem);
  double boundingFunction(gapProblem problem);
  std::vector<gapProblem> getCandidateSolutions(gapProblem problem);
  bool validateListResult(gapProblem problem, std::vector<int> solution);
};

#endif
