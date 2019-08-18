#ifndef __GAP_PROBLEM__
#define __GAP_PROBLEM__

#include <vector>
#include <iostream>
using Matrix = std::vector<std::vector<int>>;

// auxiliar operator for debugging
std::ostream& operator<<(std::ostream& os, Matrix matrix);

class gapProblem
{

public:
  Matrix JobCostPerAgent;
  Matrix ResourceConsumedPerJob;
  std::vector<int> MaximumResourcePerAgent;
  std::vector<int> CurrentResourcePerAgent;
  int numberOfJobs;
  int numberOfAgents;
  int solutionValue;

public:
  gapProblem(Matrix JobCostPerAgent_,
             Matrix ResourceConsumedPerJob_,
             std::vector<int> MaximumResourcePerAgent_) :
              JobCostPerAgent(JobCostPerAgent_),
              ResourceConsumedPerJob(ResourceConsumedPerJob_),
              MaximumResourcePerAgent(MaximumResourcePerAgent_),
              solutionValue(0)
  {
    numberOfJobs = JobCostPerAgent_[0].size();
    numberOfAgents = MaximumResourcePerAgent.size();
    CurrentResourcePerAgent.assign(numberOfAgents, 0);
  }

  void linkAgentToJob(int agent, int job){
    if(agent > numberOfAgents || job > numberOfJobs) {
      std::cerr << "Out Of Bounds " << "Agent Index value:" << agent;
      std::cerr << " Job Index value:" << job << std::endl;
      exit(-1);
    }
    solutionValue += JobCostPerAgent[agent][job];
    CurrentResourcePerAgent[agent] += ResourceConsumedPerJob[agent][job];
  }

  int agentCanDoJob(int agent, int job)
  {
    int agentResource = ResourceConsumedPerJob[agent][job] + CurrentResourcePerAgent[agent];
    return MaximumResourcePerAgent[agent] > agentResource;
  }
};

#endif