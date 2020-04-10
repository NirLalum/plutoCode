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
	// ctor
	COM();
	Result setParallelInvesrKinematics(double pxDes, double pyDes, double pzDes, double gammeDes);
	// creates a path to robot COM
	Result COMpathFunc(vector<double>& COMinitVec, vector<double>& COMfinVec); // each vec includes cmX, cmY, cmZ, gamma (the angle between z axis)
	// for forward (forward = 1) and backward (forward = -1) movement 
	Result robotBackwardForward(int forward);
	// crab walk mode (side = 1 for right movement and -1 for left)
	Result robotCrabWalk(int side);
	// turning the robot around itself (1 for +z and -1 for -z)
	Result robotTurn(int direction);
	// this func is for creating robot to world transform matrix
	Result getRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma);
	// this func caculates the inverse of a transformation matrix
	Matrix<double>& getAinv(Matrix<double>& Ain);
	// dtor
	~COM();
protected:
	Matrix<double> A_R_W{4, 4}; // curr robot to world transformation
	double currGamma; // current orientation of the robot relative to the world in radian
	vector<double> COMxVec; 
	vector<double> COMyVec;
	vector<double> COMzVec;
	vector<double> gammaVec;
	LfLeg LfLeg;
	RfLeg RfLeg;
	RbLeg RbLeg;
	LbLeg LbLeg;
};

#endif // COM_H

