#include "gapSolver.hpp"
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
    this->policy = stringToPolicy(policyStr);
    readProblemSetFile(path);
  }
  
  HeuristicPolicy gapSolver::stringToPolicy(std::string policyStr)
  {
    std::transform(policyStr.begin(), policyStr.end(), policyStr.begin(),
                   [](unsigned char c) { return std::toupper(c);});
    if (policyStr == "MAXCOST")
      return HeuristicPolicy::MAXCOST;
    if (policyStr == "MINRES")
      return HeuristicPolicy::MINRES;

    std::cerr << "Invalid policy, expected MAXCOST OR MINRES and got something else" << std::endl;
    exit(-1);
  }

  gapProblem gapSolver::branchAndBound(gapProblem problem) {
	gapProblem current_optimum = heuristicSolve(problem);
	double lower_bound = current_optimum.currentBoundingValue();
	std::queue<gapProblem> candidate_queue;
	candidate_queue = problem.getCandidateSolutions();

	while(!candidate_queue.empty()){
		auto node = candidate_queue.front();
		candidate_queue.pop();

		if(node.NotAllocatedJobs.size() == 0){
			if(node.solutionValue > lower_bound){
				current_optimum = node;
				//lower_bound = node.solutionValue;
				lower_bound = node.currentBoundingValue();
			}
		}

		else {
			auto children = node.getCandidateSolutions();
			while(!children.empty()){
				auto child = children.front();
				children.pop();
//				std::cout << child.currentBoundingValue() << std::endl;
				if(/*boundingFunction(child)*/child.currentBoundingValue() >= lower_bound){
					candidate_queue.push(child);
				}
			}
		}
	}
				std::cout << lower_bound;

	return current_optimum;
  }

  gapProblem gapSolver::heuristicSolve(gapProblem problem) {
      Matrix JobCostPerAgent = problem.JobCostPerAgent;
      for (int i = 0; i < problem.numberOfJobs; i++)
      {
        int agentIndex;
        switch(policy){
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

  void gapSolver::heuristicSolveAll()
  {
    for (gapProblem problem : problemSet)
    {
//	auto solution = heuristicSolve(problem);
	auto solution = branchAndBound(problem);
      std::cout << "Solution value " << solution.solutionValue << std::endl;
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

double gapSolver::boundingFunction(gapProblem problem){
	double boundingValue = 0;
	for(int job : problem.NotAllocatedJobs){
		for(int agent = 0 ; agent < problem.numberOfAgents; agent++){
			if (problem.CurrentResourcePerAgent[agent]
		            + problem.ResourceConsumedPerJob[agent][job]
			    < problem.MaximumResourcePerAgent[agent]){

				boundingValue += problem.JobCostPerAgent[agent][job] /problem.ResourceConsumedPerJob[agent][job];
			}
		}
	}

	return boundingValue;
}
