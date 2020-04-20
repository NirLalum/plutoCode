#include <iostream>
#include <string>
#include <stdlib.h>

#include "COM.H"
#include "RfLeg.H"
#include "LfLeg.H"

using namespace std;

int main() {
	COM robotCOM;
//	RbLeg leg;
//	leg.pathFunc(30, 0);
	int forward = 1;
	vector<double> COMinitVec { 0, 0, 0, 0 };
	vector<double> COMfinVec {0, 30, 0, 0 };

	robotCOM.robotBackwardForward(COMinitVec, COMfinVec, forward);
}	

