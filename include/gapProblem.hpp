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

  bool agentCanDoJob(int agent, int job)
  {
    if(agent < 0 || agent >= numberOfAgents || job < 0 || job >= numberOfJobs) return false;

    int agentResource = ResourceConsumedPerJob[agent][job] + CurrentResourcePerAgent[agent];
    return MaximumResourcePerAgent[agent] >= agentResource;
  }
  
  void shiftJobAgent(int agent, int job)
  {
    if(agent < 0 || agent >= numberOfAgents || job < 0 || job >= numberOfJobs)
    {
      std::cerr << "Out Of Bounds "
                << "Agent Index value:" << agent;
      std::cerr << " Job Index value:" << job << std::endl;
      exit(-1);
    }

	int oldAgent = solutionList[job];

	if(oldAgent == -1){
		linkAgentToJob(agent,job);
	}

	else {
		this->solutionValue -= JobCostPerAgent[oldAgent][job];
		this->solutionValue += JobCostPerAgent[agent][job];
		this->allocatedResources -= ResourceConsumedPerJob[oldAgent][job];
		this->allocatedResources += ResourceConsumedPerJob[agent][job];
		CurrentResourcePerAgent[agent] -= ResourceConsumedPerJob[oldAgent][job];
		CurrentResourcePerAgent[agent] += ResourceConsumedPerJob[agent][job];
		solutionList[job] = agent;
	}	
  }
 

  bool canSwapJobAgents(int firstJob, int secondJob){
    int firstJobAgent = solutionList[firstJob];
    int secondJobAgent = solutionList[secondJob];
	if(firstJobAgent == -1 or secondJobAgent == -1) return false;
    if(firstJob < 0 || firstJob >= numberOfJobs || secondJob < 0 || secondJob >= numberOfJobs){
      std::cerr << "Out Of Bounds Job" << std::endl;
      std::cerr << "First Job: " << firstJob << std::endl;
      std::cerr << "Second Job: " << secondJob << std::endl;
      exit(-1);
    }

    int firstAgentResource = CurrentResourcePerAgent[firstJobAgent] - ResourceConsumedPerJob[firstJobAgent][firstJob];
    int secondAgentResource = CurrentResourcePerAgent[secondJobAgent] - ResourceConsumedPerJob[secondJobAgent][secondJob];

    if(firstAgentResource + ResourceConsumedPerJob[firstJobAgent][secondJob] <= MaximumResourcePerAgent[firstJobAgent] &&
       secondAgentResource + ResourceConsumedPerJob[secondJobAgent][firstJob] <= MaximumResourcePerAgent[secondJobAgent]) 
       return true;

    else return false;
  }

  void swapJobAgents(int firstJob, int secondJob)
  {
    if(firstJob < 0 || firstJob >= numberOfJobs || secondJob < 0 || secondJob >= numberOfJobs)
    {
      std::cerr << "Out Of Bounds Job" << std::endl;
      std::cerr << "First Job: " << firstJob << std::endl;
      std::cerr << "Second Job: " << secondJob << std::endl;
      exit(-1);
    }

	int firstAgent = solutionList[firstJob];
    int secondAgent = solutionList[secondJob];

    this->solutionValue -= JobCostPerAgent[firstAgent][firstJob];
    this->solutionValue += JobCostPerAgent[firstAgent][secondJob];
    this->solutionValue -= JobCostPerAgent[secondAgent][secondJob];
    this->solutionValue += JobCostPerAgent[secondAgent][firstJob];

    this->allocatedResources -= ResourceConsumedPerJob[firstAgent][firstJob];
    this->allocatedResources += ResourceConsumedPerJob[firstAgent][secondJob];
    this->allocatedResources -= ResourceConsumedPerJob[secondAgent][secondJob];
    this->allocatedResources += ResourceConsumedPerJob[secondAgent][firstJob];

    CurrentResourcePerAgent[firstAgent] -= ResourceConsumedPerJob[firstAgent][firstJob];
    CurrentResourcePerAgent[firstAgent] += ResourceConsumedPerJob[firstAgent][secondJob];
    CurrentResourcePerAgent[secondAgent] -= ResourceConsumedPerJob[secondAgent][secondJob];
    CurrentResourcePerAgent[secondAgent] += ResourceConsumedPerJob[secondAgent][firstJob];
    solutionList[firstJob] = secondAgent;
    solutionList[secondJob] = firstAgent;
  }

  int currentBoundingValue()
  {
    int boundValue = this->solutionValue;
    if (NotAllocatedJobs.size() != 0)
    {
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
		boundValue += JobCostPerAgent[agentIndex][job];
	}
      }
    }
    return boundValue;
  }
};

std::ostream &operator<<(std::ostream &os, gapProblem& problem);
bool operator==(gapProblem a, gapProblem b);
#endif
