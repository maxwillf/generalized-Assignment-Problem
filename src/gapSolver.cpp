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

  std::cout << policyStr << std::endl;
  if (policyStr == "HEURISTIC")
    return SolverPolicy::HEURISTIC;
  if (policyStr == "BNB")
    return SolverPolicy::BNB;
  if (policyStr == "BT")
    return SolverPolicy::BT;
  if (policyStr == "TS")
    return SolverPolicy::TS;
  if (policyStr == "BA")
    return SolverPolicy::BA;

  std::cerr << "Invalid policy, expected HEURISTIC,BNB, or BT and got something else" << std::endl;
  exit(-1);
}

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



std::vector<gapProblem> gapSolver::getShiftNeighbors(gapProblem problem,int job){
	//int solutionSize = problem.solutionList.size();
	std::vector<gapProblem> shiftNeighbors;

	//for (int job = 0; job < solutionSize; ++job) {
		for(int agent = 0; agent < problem.numberOfAgents; ++agent){
			if(problem.agentCanDoJob(agent,job)){
			  gapProblem newProb(problem);
				if(problem.solutionList[job] != -1){
						newProb.unlinkAgentFromJob(job);
				}
			  newProb.shiftJobAgent(agent,job);
			  shiftNeighbors.push_back(newProb);
			}
		}
	//}

	return shiftNeighbors;
}

std::vector<gapProblem> gapSolver::getSwapNeighbors(gapProblem problem, int job){
	//int solutionSize = problem.solutionList.size();
	std::vector<gapProblem> swapNeighbors;

	//for (int job = 0; job < solutionSize; ++job) {
		for (int swapJob = job + 1; swapJob < problem.numberOfJobs; ++swapJob) {
			if(problem.canSwapJobAgents(job,swapJob)){
				gapProblem newProb(problem);
				newProb.swapJobAgents(job,swapJob);
				swapNeighbors.push_back(newProb);
			}
		}
	//}

	return swapNeighbors;
}

std::vector<gapProblem> gapSolver::getNeighbors(gapProblem problem, int job) {

	auto shiftNeighbors = getShiftNeighbors(problem,job);
	auto swapNeighbors = getSwapNeighbors  (problem,job);

	swapNeighbors.insert(swapNeighbors.end(),shiftNeighbors.begin(),shiftNeighbors.end());
	return swapNeighbors;
}

std::vector<gapProblem> gapSolver::getAllNeighbors(gapProblem problem) {

  std::vector<gapProblem> neighbors;
  for (size_t i = 0; i < problem.numberOfJobs; i++)
  {
    auto shiftNeighbors = getShiftNeighbors(problem,i);
    auto swapNeighbors = getSwapNeighbors  (problem,i);
    neighbors.insert(neighbors.end(),shiftNeighbors.begin(), shiftNeighbors.end());
    neighbors.insert(neighbors.end(),swapNeighbors.begin(), swapNeighbors.end());
    /* code */
  }
  
	return neighbors;
}

gapProblem gapSolver::tabuSearch(gapProblem problem)
{
  int maximumTabuListLength = 50;
  int iterationsWithoutNewBest = 0;
  gapProblem best = getBestHeuristicSolution(problem); 
  auto solutionList = problem.solutionList;
  if(std::any_of(solutionList.begin(),solutionList.end(),[solutionList](int x){
    return x != -1;
  })){
    best = problem;
  }
  gapProblem candidateBest = best;
  std::deque<gapProblem> linearTabuList;
  linearTabuList.push_back(best);
  std::vector<gapProblem> neighbors;
  std::vector<gapProblem> longTermMemory;

  while(iterationsWithoutNewBest <= 50){
	  for (int job = 0; job < problem.numberOfJobs; ++job) {
		  neighbors = getNeighbors(candidateBest,job);
		  if (neighbors.empty()) {
			  continue;
		  }
			std::vector<gapProblem> augmentingCandidates;
			std::copy_if(neighbors.begin(),neighbors.end(),std::back_inserter(augmentingCandidates),
							[best](gapProblem p){return p.solutionValue > best.solutionValue; });
	
			if(!augmentingCandidates.empty()){
					auto maxElemIter = std::max_element(augmentingCandidates.begin(),augmentingCandidates.end(),
					  [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue; });
					auto maxElem = *maxElemIter;
					augmentingCandidates.erase(maxElemIter);
					// keep other possible solutions in longTermMemory in order to escape local
					// minima
					if(!augmentingCandidates.empty()){
							longTermMemory.insert(longTermMemory.end(),augmentingCandidates.begin(),augmentingCandidates.end());
					}
					
					if(std::find(linearTabuList.begin(),linearTabuList.end(),maxElem) == linearTabuList.end()){
							if(linearTabuList.size() >= 50){
									linearTabuList.pop_front();
									linearTabuList.push_back(maxElem);
							}
							if(maxElem.solutionValue > best.solutionValue){
									candidateBest = maxElem;
									best = maxElem;
									iterationsWithoutNewBest = 0;
									//std::cout << best << std::endl;
									break;
							}
							else {
									iterationsWithoutNewBest++;

							}
					}
			}
			else {
					iterationsWithoutNewBest++;
			}
	  }
		if(!longTermMemory.empty()){
					auto maxElemIter = std::max_element(longTermMemory.begin(),longTermMemory.end(),
					  [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue; });
				candidateBest = *maxElemIter;
				longTermMemory.erase(maxElemIter);
		}
  }
 // std::cout << best;
  return best;
}

gapProblem gapSolver::randomEjectChain(gapProblem problem, int chainLength)
{
  auto prob = problem;
  for (size_t i = 0; i < chainLength ; i++)
  {
    int job = rand() % problem.numberOfJobs;
    auto neighbors = getNeighbors(prob,job);

    if(!neighbors.empty()){
      //if(i+1 == chainLength ) {
      //  return *std::max_element(neighbors.begin(),neighbors.end(),
      //  [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue; });
      //}
      prob = neighbors[rand() % neighbors.size()];
      //prob = *std::max_element(neighbors.begin(),neighbors.end(),
      //  [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue; });

    }
  }
  return prob;
}

gapProblem gapSolver::beesAlgorithm(gapProblem problem)
{
  int numberOfBestBees = 5; 
  gapProblem currrentSolution = getBestHeuristicSolution(problem);
  std::vector<gapProblem> neighbors; 
  for (size_t i = 0; i < numberOfBestBees; i++)

  {
    neighbors.insert(neighbors.end(),randomEjectChain(currrentSolution, rand() % problem.numberOfJobs));
  }

  int iteration = 0;
  int numberOfIterationsWithoutUpdatesBeforeRandomization = 50;
  int currentRandomizerIteration = 0;
  int maxRandomizerIterations = 3;
  int chainLength = 5;

  while(currentRandomizerIteration < maxRandomizerIterations){
    std::sort(neighbors.begin(),neighbors.end(), [](gapProblem a, gapProblem b){ return a.solutionValue > b.solutionValue; });
    for (auto && bee : neighbors)
    {
      auto currentBeeNeighbors = getAllNeighbors(bee);
      // should be number of onlookers
      for (size_t i = 0; i < numberOfBestBees; i++)
      {
        currentBeeNeighbors.insert(currentBeeNeighbors.end(), randomEjectChain(bee,numberOfBestBees));
      }
      
      auto max = *std::max_element(currentBeeNeighbors.begin(),currentBeeNeighbors.end(),
      [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue;});
      if(bee < max){
        bee = max;
      }
    }
    auto possibleSolution = *std::max_element(neighbors.begin(),neighbors.end());
    if(currrentSolution.solutionValue < possibleSolution.solutionValue){
      currrentSolution = possibleSolution;
      iteration = 0;
    }
    if(iteration == numberOfIterationsWithoutUpdatesBeforeRandomization){
      //auto minElem = std::min_element(neighbors.begin(),neighbors.end(),
      //[](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue;});

      for (size_t i = 0; i < neighbors.size() / 2; i++)
      {
        auto minElem = neighbors.back();
        neighbors.pop_back();
        neighbors.push_back(tabuSearch(randomEjectChain(minElem,chainLength)));
      }
      
      //*minElem = randomEjectChain(*minElem,chainLength);
      //*minElem = tabuSearch(randomEjectChain(*minElem,chainLength));
      currentRandomizerIteration++;
      iteration = 0;
    }
    else {
      iteration++;
    }
  } 
  //std::cout << solution;
  return currrentSolution;
  //randomEjectChain(initialGuess, 5);
  
					//auto maxElemIter = std::max_element(augmentingCandidates.begin(),augmentingCandidates.end(),
					//  [](gapProblem a, gapProblem b){ return a.solutionValue < b.solutionValue; });
  
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
      std::cerr << "No defined heuristic policy error" << std::endl;
      exit(-1);
    }
    if (agentIndex != -1)
    {
      problem.linkAgentToJob(agentIndex, i);
    }
  }

  return problem;
}

gapProblem gapSolver::getBestHeuristicSolution(gapProblem problem){
  this->heuristicPolicy = HeuristicPolicy::MAXCOST;
  gapProblem maxres = heuristicSolve(problem);
  this->heuristicPolicy = HeuristicPolicy::MINRES;
  gapProblem minres = heuristicSolve(problem);
  return maxres.solutionValue > minres.solutionValue ? maxres : minres;
}

void gapSolver::solveAll()
{
//  gapProblem (gapSolver::* solveFunc)(gapProblem);
	std::function<gapProblem(gapSolver&,gapProblem)> solveFunc;
	switch(solverPolicy){
	  case SolverPolicy::HEURISTIC:
		  solveFunc =  &gapSolver::getBestHeuristicSolution;
      break;
	  case SolverPolicy::BNB:
		  solveFunc =  &gapSolver::branchAndBound;
      break;
	  case SolverPolicy::BT:
		  solveFunc =  &gapSolver::backTracking;
      break;
	  case SolverPolicy::TS:
		  solveFunc =  &gapSolver::tabuSearch;
      break;
	  case SolverPolicy::BA:
		  solveFunc =  &gapSolver::beesAlgorithm;
      break;
  }

  for (gapProblem problem : problemSet)
  {
    auto solution = solveFunc(*this,problem);
    //auto solution = (this->*solveFunc)(problem);
    std::cout << solution;
    validateListResult(solution,solution.solutionList);
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
    problem.solutionList.assign(solutionSize,-1);

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
