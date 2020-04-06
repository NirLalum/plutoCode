#ifndef COM_H
#define COM_H

#include <vector>
#include <math.h>
#include "GenericMatrix.H"
#include "Definitions.H"
#include "LfLeg.H"
#include "RfLeg.H"
#include "RbLeg.H"
#include "LbLeg.H"

class COM {
public:
	COM();
	Result setParallelInvesrKinematics(double px, double py, double pz, double desGamme);
	// creates a path to robot COM
	Result COMpathFunc(vector<double>& COMinitVec, vector<double>& COMfinVec); // each vec includes cmX, cmY, cmZ, gamma (the angle between z axis)
	Result robotBackwardForward(int forward);
	Result robotCrabWalk(int side);
	Result robotTurn(int direction);
	Result getRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma);
	~COM();
protected:
	Matrix<double> A_R_W{4, 4}; // curr robot to world transformation
	double currGamma; // current orientation of the robot relative to the world in radian
	vector<double> COMxVec; 
	vector<double> COMyVec;
	vector<double> COMzVec;
	LfLeg LfLeg;
	RfLeg RfLeg;
	RbLeg RbLeg;
	LbLeg LbLeg;
};


#endif // COM_H

