#ifndef __GAP_SOLVER__
#define __GAP_SOLVER__

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <cctype>
#include <deque>
#include "gapProblem.hpp"

#define INF ((unsigned) ~0)

enum class HeuristicPolicy {MAXCOST,MINRES};
enum class SolverPolicy {HEURISTIC,BNB,BT,TS};

class gapSolver
{

private:
  std::vector<gapProblem> problemSet;
  SolverPolicy solverPolicy;
  HeuristicPolicy heuristicPolicy;
  
  SolverPolicy stringToPolicy(std::string policyStr);
//  HeuristicPolicy stringToPolicy(std::string policyStr);
  std::vector<gapProblem> getShiftNeighbors(gapProblem problem, int job);
  std::vector<gapProblem> getSwapNeighbors (gapProblem problem, int job);
  std::vector<gapProblem> getNeighbors     (gapProblem problem, int job);
  void heuristicSolveAll(); 
  void backTrackingSolveAll();
  void branchAndBoundSolveAll();
  int getMinimumResourceCostAgent(int job, gapProblem problem);
  int getMaximumCostAvailableAgent(int job, gapProblem problem);
  double boundingFunction(gapProblem problem);
  std::vector<gapProblem> getCandidateSolutions(gapProblem problem);
  bool validateListResult(gapProblem problem, std::vector<int> solution);
  gapProblem getBestHeuristicSolution(gapProblem problem);
  gapProblem heuristicSolve(gapProblem problem);
  gapProblem branchAndBound(gapProblem problem);
  gapProblem backTracking(gapProblem problem);
  gapProblem tabuSearch(gapProblem problem);

public:
  void readProblemSetFile(std::string path);

  gapSolver();
  gapSolver(std::string path, std::string Policy ="MAXCOST");

  void solveAll(); 
};

#endif
