#include "gapSolver.hpp"
#include <functional>
void gapSolver::readProblemSetFile(std::string path)
{
  std::fstream fs(path);

  int problemSetSize = 0;
  int currentProblem = 0;

  fs >> problemSetSize;

  //std::cout << problemSetSize << std::endl;

  while (currentProblem < problemSetSize)
  {
    int numberOfAgents;
    int numberOfJobs;
    int currentNumber;
    fs >> numberOfAgents >> numberOfJobs;
    //std::cout << numberOfAgents << " " << numberOfJobs << std::endl;
    Matrix JobCostPerAgent(numberOfAgents);

    for (int i = 0; i < numberOfAgents; ++i)
    {
      for (int j = 0; j < numberOfJobs; ++j)
      {
        fs >> currentNumber;
        JobCostPerAgent[i].push_back(currentNumber);
      }
    }
    //std::cout << JobCostPerAgent;

    Matrix ResourceConsumed(numberOfAgents);

    for (int i = 0; i < numberOfAgents; ++i)
    {
      for (int j = 0; j < numberOfJobs; ++j)
      {
        fs >> currentNumber;
        ResourceConsumed[i].push_back(currentNumber);
      }
    }

    std::vector<int> MaximumResourcePerAgent;
    for (int i = 0; i < numberOfAgents; ++i)
    {
      fs >> currentNumber;
      MaximumResourcePerAgent.push_back(currentNumber);
    }
    problemSet.push_back(gapProblem(JobCostPerAgent, ResourceConsumed, MaximumResourcePerAgent));

    for (auto it = MaximumResourcePerAgent.begin(); it != MaximumResourcePerAgent.end(); ++it)
    {
      // std::cout << *it << " ";
    }
    //std::cout << std::endl;
    currentProblem++;
  }
}

gapSolver::gapSolver() {}

gapSolver::gapSolver(std::string path, std::string policyStr)
{
  this->solverPolicy = stringToPolicy(policyStr);
  readProblemSetFile(path);
}

SolverPolicy gapSolver::stringToPolicy(std::string policyStr)
{
  std::transform(policyStr.begin(), policyStr.end(), policyStr.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  if (policyStr == "HEURISTIC")
    return SolverPolicy::HEURISTIC;
  if (policyStr == "BNB")
    return SolverPolicy::BNB;
  if (policyStr == "BT")
    return SolverPolicy::BT;

  std::cerr << "Invalid policy, expected MAXCOST OR MINRES and got something else" << std::endl;
  exit(-1);
}
/*
HeuristicPolicy gapSolver::stringToPolicy(std::string policyStr)
{
  std::transform(policyStr.begin(), policyStr.end(), policyStr.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  if (policyStr == "MAXCOST")
    return HeuristicPolicy::MAXCOST;
  if (policyStr == "MINRES")
    return HeuristicPolicy::MINRES;

  std::cerr << "Invalid policy, expected MAXCOST OR MINRES and got something else" << std::endl;
  exit(-1);
}*/

gapProblem gapSolver::backTracking(gapProblem problem)
{
  std::vector<gapProblem> candidate_queue;
  candidate_queue = getCandidateSolutions(problem);
  gapProblem current_optimum = problem;
  int amountOfVerifiedNodes = 0;
  current_optimum.solutionValue = 0;
  while (!candidate_queue.empty())
  {
    auto node = candidate_queue.back();
    candidate_queue.pop_back();

    auto children = getCandidateSolutions(node);

    if (children.size() == 0)
    {
      if (node.solutionValue > current_optimum.solutionValue)
      {
        current_optimum = node;
        amountOfVerifiedNodes++;
      }

    }

    else
    {
      while (!children.empty())
      {
        auto child = children.back();
        children.pop_back();
        amountOfVerifiedNodes++;
        candidate_queue.push_back(child);
      }
    }
  }
  std::cout << current_optimum;
  return current_optimum;
}


gapProblem gapSolver::branchAndBound(gapProblem problem)
{
  gapProblem maxres = heuristicSolve(problem);
  this->heuristicPolicy = HeuristicPolicy::MINRES;
  gapProblem minres = heuristicSolve(problem);
  gapProblem current_optimum = maxres.solutionValue > minres.solutionValue ? maxres : minres;
  int lower_bound = current_optimum.currentBoundingValue();
  std::vector<gapProblem> candidate_queue;
  candidate_queue = getCandidateSolutions(problem);
  int amountOfVerifiedNodes = 0;

  //std::cout << current_optimum;

  while (!candidate_queue.empty())
  {
    auto node = candidate_queue.back();
    candidate_queue.pop_back();

    auto children = getCandidateSolutions(node);

    if (children.size() == 0)
    {
      if (node.solutionValue > current_optimum.solutionValue)
      {
        current_optimum = node;
        lower_bound = node.currentBoundingValue();
        amountOfVerifiedNodes++;
      }
    }

    else
    {
      while (!children.empty())
      {
        auto child = children.back();
        children.pop_back();
        amountOfVerifiedNodes++;
        int currentChildBoudingValue = child.currentBoundingValue();
        if (currentChildBoudingValue >= lower_bound)
        {
          candidate_queue.push_back(child);
        }
      }
    }
  }
  std::cout << "Amount of Nodes Verified " << amountOfVerifiedNodes << std::endl;
  std::cout << "Current lower bound Value " << lower_bound << std::endl;
  std::cout << "Size of candidate queue " << candidate_queue.size() << std::endl;
  std::cout << current_optimum;
  return current_optimum;
}

gapProblem gapSolver::heuristicSolve(gapProblem problem)
{
  Matrix JobCostPerAgent = problem.JobCostPerAgent;
  for (int i = 0; i < problem.numberOfJobs; i++)
  {
    int agentIndex = -1;
    switch (heuristicPolicy)
    {
    case HeuristicPolicy::MAXCOST:
      agentIndex = getMaximumCostAvailableAgent(i, problem);
      break;
    case HeuristicPolicy::MINRES:
      agentIndex = getMinimumResourceCostAgent(i, problem);
      break;
    default:
      std::cerr << "No defined policy error" << std::endl;
      exit(-1);
    }
    if (agentIndex != -1)
    {
      problem.linkAgentToJob(agentIndex, i);
    }
  }

  return problem;
}

void gapSolver::solveAll()
{
//  gapProblem (gapSolver::* solveFunc)(gapProblem);
	std::function<gapProblem(gapSolver&,gapProblem)> solveFunc;
	switch(solverPolicy){
	  case SolverPolicy::HEURISTIC:
		  solveFunc =  &gapSolver::heuristicSolve;
	  case SolverPolicy::BNB:
		  solveFunc =  &gapSolver::branchAndBound;
	  case SolverPolicy::BT:
		  solveFunc =  &gapSolver::backTracking;
  }

  for (gapProblem problem : problemSet)
  {
    auto solution = solveFunc(*this,problem);
    //auto solution = (this->*solveFunc)(problem);
    validateListResult(solution,solution.solutionList);
  }
}

void gapSolver::branchAndBoundSolveAll()
{
  for (gapProblem problem : problemSet)
  {
    //	auto solution = heuristicSolve(problem);
    auto solution = branchAndBound(problem);
    validateListResult(solution,solution.solutionList);
  }
}

void gapSolver::backTrackingSolveAll()
{
  for (gapProblem problem : problemSet)
  {
    //	auto solution = heuristicSolve(problem);
     backTracking(problem);
    //auto solution = backTracking(problem);
    //validateListResult(solution,solution.solutionList);
  }
}

int gapSolver::getMinimumResourceCostAgent(int job, gapProblem problem)
{
  unsigned int minimum = INF;
  int minimumIndex = -1;
  for (int i = 0; i < problem.numberOfAgents; i++)
  {
    if (minimum > problem.ResourceConsumedPerJob[i][job])
    {
      if (problem.agentCanDoJob(i, job))
      {
        minimumIndex = i;
        minimum = problem.ResourceConsumedPerJob[i][job];
      }
    }
  }
  return minimumIndex;
}

int gapSolver::getMaximumCostAvailableAgent(int job, gapProblem problem)
{
  int maximum = -1;
  int maximumIndex = -1;
  for (int i = 0; i < problem.numberOfAgents; i++)
  {
    if (maximum < problem.JobCostPerAgent[i][job])
    {
      if (problem.agentCanDoJob(i, job))
      {
        maximumIndex = i;
        maximum = problem.JobCostPerAgent[i][job];
      }
    }
  }
  return maximumIndex;
}

std::vector<gapProblem> gapSolver::getCandidateSolutions(gapProblem problem)
{
  std::vector<gapProblem> candidates;
  int jobIndex = 
    std::distance(problem.solutionList.begin(),
                  std::find(problem.solutionList.begin(),problem.solutionList.end(), -1)
    );
  if(jobIndex < problem.numberOfJobs) {
	  for (int agent = 0; agent < problem.numberOfAgents; agent++)
	  {
		  if (problem.agentCanDoJob(agent, jobIndex))
		  {
			  gapProblem newProb(problem);
			  newProb.linkAgentToJob(agent, jobIndex);
			  candidates.push_back(newProb);
		  }
	  }
  }
  return candidates;
}

  bool gapSolver::validateListResult(gapProblem problem, std::vector<int> solution){
    int oldSolutionValue = problem.solutionValue;
    int solutionSize = solution.size();
    problem.CurrentResourcePerAgent.assign(problem.numberOfAgents,0);
    problem.solutionValue = 0;

    for (size_t i = 0; i < solutionSize ; i++)
    {
      if (problem.agentCanDoJob(solution[i],i)){
        problem.linkAgentToJob(solution[i],i);
      }
    }
    std::cout << (problem.solutionValue == oldSolutionValue ?
     "Validado" : "Não Validado") << std::endl;
     //std::cout << problem.solutionValue << std::endl;
    return problem.solutionValue == oldSolutionValue;
}
