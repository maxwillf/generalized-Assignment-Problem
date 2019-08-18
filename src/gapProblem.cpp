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