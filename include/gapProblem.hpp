#ifndef __GAP_PROBLEM__
#define __GAP_PROBLEM__

#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>

using Matrix = std::vector<std::vector<int>>;

// auxiliar operator for debugging
std::ostream &operator<<(std::ostream &os, Matrix matrix);

class gapProblem
{

public:
  Matrix JobCostPerAgent;
  Matrix ResourceConsumedPerJob;
  std::vector<int> MaximumResourcePerAgent;
  std::vector<int> CurrentResourcePerAgent;
  std::vector<int> NotAllocatedJobs;
  std::vector<int> solutionList;
  int numberOfJobs;
  int numberOfAgents;
  int solutionValue;
  int allocatedResources;

public:
  gapProblem()
  {
  }

  gapProblem(Matrix JobCostPerAgent_,
             Matrix ResourceConsumedPerJob_,
             std::vector<int> MaximumResourcePerAgent_) : JobCostPerAgent(JobCostPerAgent_),
                                                          ResourceConsumedPerJob(ResourceConsumedPerJob_),
                                                          MaximumResourcePerAgent(MaximumResourcePerAgent_),
                                                          solutionValue(0),
                                                          allocatedResources(0)
  {
    numberOfJobs = JobCostPerAgent_[0].size();
    numberOfAgents = MaximumResourcePerAgent.size();
    CurrentResourcePerAgent.assign(numberOfAgents, 0);
    solutionList.assign(numberOfJobs, -1);

    for (int i = 0; i < numberOfJobs; i++)
    {
      NotAllocatedJobs.push_back(i);
    }
  }

  gapProblem(const gapProblem &problem)
  {
    *this = problem;
  }

  void linkAgentToJob(int agent, int job)
  {
    if (agent > numberOfAgents || job > numberOfJobs)
    {
      std::cerr << "Out Of Bounds "
                << "Agent Index value:" << agent;
      std::cerr << " Job Index value:" << job << std::endl;
      exit(-1);
    }

    this->solutionValue += JobCostPerAgent[agent][job];
    this->allocatedResources += ResourceConsumedPerJob[agent][job];
    CurrentResourcePerAgent[agent] += ResourceConsumedPerJob[agent][job];
    solutionList[job] = agent;
    auto it = std::find(NotAllocatedJobs.begin(), NotAllocatedJobs.end(), job);
    if (it != NotAllocatedJobs.end())
    {
      NotAllocatedJobs.erase(it);
    }
  }

  int agentCanDoJob(int agent, int job)
  {
    if(agent < 0 || agent >= numberOfAgents || job < 0 || job >= numberOfJobs) return false;

    int agentResource = ResourceConsumedPerJob[agent][job] + CurrentResourcePerAgent[agent];
    return MaximumResourcePerAgent[agent] >= agentResource;
  }

  int currentBoundingValue()
  {
    int boundValue = this->solutionValue;
    //for (int i = 0; i < numberOfJobs; i++)
    //{
    //  int agentIndex = solutionList[i];
    //  if (agentIndex != -1)
    //  {
    //    boundValue += JobCostPerAgent[agentIndex][i];
    //  }
    //}

    if (NotAllocatedJobs.size() != 0)
    {
    //  gapProblem gapCopy = *this;
      for (int job : NotAllocatedJobs)
      {
        std::vector<int> bounds;
        for (int i = 0; i < numberOfAgents; i++)
        {
          if (agentCanDoJob(i, job))
          {
            bounds.push_back(JobCostPerAgent[i][job]);
          }
          else
          {
            bounds.push_back(0);
          }
        }
        auto maxElem = std::max_element(bounds.begin(), bounds.end());
	if(*maxElem > 0){
		int agentIndex = maxElem - bounds.begin();
		//gapCopy.linkAgentToJob(agentIndex,job);
		boundValue += JobCostPerAgent[agentIndex][job];
	}
      }
      //boundValue = gapCopy.solutionValue;
    }
    return boundValue;
  }
};

std::ostream &operator<<(std::ostream &os, gapProblem& problem);
#endif
