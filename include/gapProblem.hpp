#ifndef __GAP_PROBLEM__
#define __GAP_PROBLEM__

#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>

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
  std::vector<int> NotAllocatedJobs;
  int numberOfJobs;
  int numberOfAgents;
  int solutionValue;

public:
  gapProblem () {
  }

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

    for(int i = 0; i < numberOfJobs; i++) {
	NotAllocatedJobs.push_back(i);
    }
  }

  gapProblem(const gapProblem& problem) {
	*this = problem;
  }

  void linkAgentToJob(int agent, int job){
    if(agent > numberOfAgents || job > numberOfJobs) {
      std::cerr << "Out Of Bounds " << "Agent Index value:" << agent;
      std::cerr << " Job Index value:" << job << std::endl;
      exit(-1);
    }
    solutionValue += JobCostPerAgent[agent][job];
    CurrentResourcePerAgent[agent] += ResourceConsumedPerJob[agent][job];
    auto it =  std::find(NotAllocatedJobs.begin(), NotAllocatedJobs.end(),job);
    if(it != NotAllocatedJobs.end()){
    NotAllocatedJobs.erase(it);
  	}
  }

  int agentCanDoJob(int agent, int job)
  {
    int agentResource = ResourceConsumedPerJob[agent][job] + CurrentResourcePerAgent[agent];
    return MaximumResourcePerAgent[agent] > agentResource;
  }

  std::queue<gapProblem> getCandidateSolutions() {
	std::queue<gapProblem> candidates;

	for(int job : NotAllocatedJobs){
		for(int agent = 0 ; agent < numberOfAgents; agent++){
			if (CurrentResourcePerAgent[agent] + ResourceConsumedPerJob[agent][job] < MaximumResourcePerAgent[agent]){
				gapProblem newProb(*this);
				newProb.linkAgentToJob(agent,job);
				candidates.push(newProb);
			}
		}
	}
	return candidates;
  }

  double currentBoundingValue(){
	  double boundingValue = 0;
		double sumOfResources = 0;
		
		for(int i = 0; i < numberOfAgents; i++){
			sumOfResources += CurrentResourcePerAgent[i];		
		}

		boundingValue = solutionValue / sumOfResources;

	if(NotAllocatedJobs.size() != 0){
		for(int job : NotAllocatedJobs){
			std::vector<double>  bounds;
			for(int i = 0; i < numberOfAgents; i++){
				bounds.push_back(JobCostPerAgent[i][job]/ResourceConsumedPerJob[i][job]);
			}
			boundingValue += *std::max_element(bounds.begin(), bounds.end());
		}
	}
	return boundingValue;
  }
};

#endif
