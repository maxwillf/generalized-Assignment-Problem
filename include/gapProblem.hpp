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
  std::vector<int> solutionList;
  int numberOfJobs;
  int numberOfAgents;
  int solutionValue;
  int allocatedResources;

public:
  gapProblem () {
  }

  gapProblem(Matrix JobCostPerAgent_,
             Matrix ResourceConsumedPerJob_,
             std::vector<int> MaximumResourcePerAgent_) :
              JobCostPerAgent(JobCostPerAgent_),
              ResourceConsumedPerJob(ResourceConsumedPerJob_),
              MaximumResourcePerAgent(MaximumResourcePerAgent_),
              solutionValue(0),
	      allocatedResources(0)
  {
    numberOfJobs = JobCostPerAgent_[0].size();
    numberOfAgents = MaximumResourcePerAgent.size();
    CurrentResourcePerAgent.assign(numberOfAgents, 0);
    solutionList.assign(numberOfJobs, 0);

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

    this->solutionValue += JobCostPerAgent[agent][job];
    this->allocatedResources += ResourceConsumedPerJob[agent][job];
    CurrentResourcePerAgent[agent] += ResourceConsumedPerJob[agent][job];
    solutionList[job] = agent;
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

  std::vector<gapProblem> getCandidateSolutions() {
	std::vector<gapProblem> candidates;

	for(int job : NotAllocatedJobs){
		for(int agent = 0 ; agent < numberOfAgents; agent++){
			if (agentCanDoJob(agent,job)){
				gapProblem newProb(*this);
				newProb.linkAgentToJob(agent,job);
				candidates.push_back(newProb);
			}
		}
	}
	return candidates;
  }

  double currentBoundingValue(){
	  int currentSolutionValue = this->solutionValue;
//	  double boundingValue = 0;
	double sumOfResources = this->allocatedResources;

//		boundingValue = solutionValue / sumOfResources;

	if(NotAllocatedJobs.size() != 0){

		for(int job : NotAllocatedJobs){
			std::vector<double>  bounds;
			for(int i = 0; i < numberOfAgents; i++){
				if(agentCanDoJob(i,job)){
				bounds.push_back(JobCostPerAgent[i][job]/ResourceConsumedPerJob[i][job]);
				}
				else {
					bounds.push_back(0);
				}
			}
			int agentIndex = std::max_element(bounds.begin(), bounds.end()) - bounds.begin();
			sumOfResources += ResourceConsumedPerJob[agentIndex][job];
			currentSolutionValue += JobCostPerAgent[agentIndex][job];
			
		}
	}
	return currentSolutionValue / sumOfResources;
  }
};

std::ostream& operator<<(std::ostream& os, gapProblem problem);
#endif
