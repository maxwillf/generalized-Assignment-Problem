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

std::ostream& operator<<(std::ostream& os, gapProblem problem){
	auto solutionSize = problem.solutionList.size();
	std::cout << "printing solution array" << std::endl;
	std::cout << "[";

	for (int i = 0; i < solutionSize - 1 ; ++i) {
		std:: cout << problem.solutionList[i] << ",";
	}
	std::cout << problem.solutionList[solutionSize -1] << "]" << std::endl;
	
	std::cout << "Bounding Value: " <<  problem.currentBoundingValue() << std::endl;
	std::cout << "Solution Value: " <<  problem.solutionValue << std::endl;
}
