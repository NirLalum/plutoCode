#include <iostream>
#include <string>
#include <stdlib.h>

#include "COM.H"
#include "RfLeg.H"
#include "LfLeg.H"

using namespace std;

int main() {
	COM robotCOM;
	RbLeg leg;
//	leg.pathFunc(0, -30);
	double forward = 1;
	double side = -1;
	/*
	vector<double> COMinitVec { 0, 0, 0, 0 };
	vector<double> COMfinVec {0, forward*30, 0, 0 };
	robotCOM.robotBackwardForward(COMinitVec, COMfinVec, forward);
	vector<double> COMinitVec2{ 0, 30, 0, 0 };
	vector<double> COMfinVec2{ 0, forward * 60, 0, 0 };
	robotCOM.robotBackwardForward(COMinitVec2, COMfinVec2, forward);
	*/

//	vector<double> COMinitVec { 0, 0, 0, 0 };
//	vector<double> COMfinVec {side*10, 0, 0, 0 };
//	robotCOM.robotCrabWalk(COMinitVec, COMfinVec, side);
	
	robotCOM.robotTurn(0, -PI/10, 0, 0, 0);


}	

