#include "gapProblem.hpp"

std::ostream& operator<<(std::ostream& os, Matrix matrix){
	int row = matrix.size();
	for (int i = 0; i < row ; ++i) {
		int column = matrix[i].size();
		for (int j = 0; j < column; ++j) {
			os << matrix[i][j] << " ";	
		}
		os << std::endl;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os,  gapProblem& problem){
	auto solutionSize = problem.solutionList.size();
	os << "printing solution array" << std::endl;
	os << "[";

	for (int i = 0; i < solutionSize - 1 ; ++i) {
		os << problem.solutionList[i] << ",";
	}
	os << problem.solutionList[solutionSize -1] << "]" << std::endl;
	os << "Bounding Value: " <<  problem.currentBoundingValue() << std::endl;
	os << "Solution Value: " <<  problem.solutionValue << std::endl;
}

bool operator==(gapProblem a, gapProblem b){
	a.solutionList == b. solutionList;
}
