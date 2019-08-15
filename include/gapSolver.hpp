#include <iostream>
#include <fstream>
#include <sstream>
#include "gapProblem.hpp"

class gapSolver
{

private:
  std::vector<gapProblem> problemSet;

public:
  void readProblemSetFile(std::string path)
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
  
  gapSolver() {}
  gapSolver(std::string path) {
    readProblemSetFile(path);  
  }
  void solveFirstProblem(){
    gapProblem problem = problemSet[0];
    Matrix JobCostPerAgent = problem.JobCostPerAgent;
     for (int i = 0; i < problem.numberOfJobs; i++)
    {
      int agentIndex = getMaximumCostAvailableAgent(i,problem);
      if(agentIndex != -1){
      problem.linkAgentToJob(agentIndex,i);
      }
    }
    std::cout << problem.solutionValue << std::endl;
  }

  int getMaximumCostAvailableAgent(int job, gapProblem problem){
    int maximum = -1;
    int maximumIndex = -1;
    for (int i = 0; i < problem.numberOfAgents; i++)
    {
        if(maximum < problem.JobCostPerAgent[i][job]){
          if(problem.agentCanDoJob(i,job)){
            maximumIndex = i;
            maximum = problem.JobCostPerAgent[i][job];
        }
      }
    }
    std::cout << problem.JobCostPerAgent << std::endl;
    std::cout << maximumIndex << std::endl;
    std::cout << maximum << std::endl;
    return maximumIndex;
  }
};
