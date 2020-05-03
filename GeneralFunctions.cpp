#include "GeneralFunctions.H"

Result linspace(vector<double>& Vec, double InitVal, double FinVal, int stepsNum) {
	if (stepsNum < 2) return FAIL;
	int i = 1;
	Vec.push_back(InitVal);
	for (i = 1; i < stepsNum; i++) {
		Vec.push_back(Vec[i - 1] + (FinVal - InitVal) / (stepsNum - 1));
	}
	return SUCCESS;
}