#include "gapSolver.hpp"
  void gapSolver::readProblemSetFile(std::string path)
  {
    std::ifstream ifs(path);
    std::string line;
    std::stringstream ss;

    int problemSetSize = 0;
    int currentProblem = 0;

    getline(ifs, line);
    ss << line;
    ss >> problemSetSize;
    //std::cout << problemSetSize << std::endl;

    while (currentProblem < problemSetSize)
    {
      int numberOfAgents;
      int numberOfJobs;
      int currentNumber;
      getline(ifs, line);
      ss = std::stringstream(line);
      ss >> numberOfAgents >> numberOfJobs;
      //std::cout << numberOfAgents << " " << numberOfJobs << std::endl;
      Matrix JobCostPerAgent(numberOfAgents);

      for (int i = 0; i < numberOfAgents; ++i)
      {
        getline(ifs, line);
        ss = std::stringstream(line);
        //ss << line;
        for (int j = 0; j < numberOfJobs; ++j)
        {
          ss >> currentNumber;
          JobCostPerAgent[i].push_back(currentNumber);
        }
      }
      //std::cout << JobCostPerAgent;

      Matrix ResourceConsumed(numberOfAgents);

      for (int i = 0; i < numberOfAgents; ++i)
      {
        getline(ifs, line);
        ss = std::stringstream(line);
        //ss << line;
        for (int j = 0; j < numberOfJobs; ++j)
        {
          ss >> currentNumber;
          ResourceConsumed[i].push_back(currentNumber);
        }
      }

      //std::cout << ResourceConsumed;
      getline(ifs, line);
      ss = std::stringstream(line);
      std::vector<int> MaximumResourcePerAgent;
      for (int i = 0; i < numberOfAgents; ++i)
      {
        ss >> currentNumber;
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

  void gapSolver::solve()
  {
    for (gapProblem problem : problemSet)
    {
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
      std::cout << "Solution value " << problem.solutionValue << std::endl;
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
    //std::cout << problem.JobCostPerAgent << std::endl;
    //std::cout << minimumIndex << std::endl;
    //std::cout << minimum << std::endl;
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
    //std::cout << problem.JobCostPerAgent << std::endl;
    //std::cout << maximumIndex << std::endl;
    //std::cout << maximum << std::endl;
    return maximumIndex;
  }

