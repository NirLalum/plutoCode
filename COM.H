#ifndef COM_H
#define COM_H

#include <vector>
#include <math.h>
#include <fstream>
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
	// postion the legs in starting position
	Result initialRobot();
	// move the robot COM according to pre made pathes in parallel inverse kinematics function
	Result moveCOM();
	Result setParallelInverseKinematics(double pxDes, double pyDes, double pzDes, double gammaDes);
	Result calcOneLegParallelInvesrKinematics(Leg* currLeg, vector<double>& CMtoShValues, double pxDes, double pyDes, double pzDes, double gammaDes);
	// creates a path to robot COM
	Result COMpathFunc(double cmXinit, double cmYinit, double cmZinit, double gammaInit, double cmXfin, double cmYfin, double cmZfin, double gammaFin); // each vec includes cmX, cmY, cmZ, gamma (the angle between z axis)
	// for forward (forward = 1) and backward (forward = -1) movement 
	Result robotBackwardForward(vector<double>& COMinitVec, vector<double>& COMfinVec,int forward);
	// crab walk mode (side = 1 for right movement and -1 for left)
	Result robotCrabWalk(vector<double>& COMinitVec, vector<double>& COMfinVec ,int side);
	// turning the robot around itself (1 for +z and -1 for -z)
	Result robotTurn(double gammaInit, double gammaFin, double currX, double currY, double currZ);
	// this func is for creating robot to world transform matrix
	Result setRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma);
	// this func caculates the inverse of a transformation matrix
	Result getAinv(Matrix<double>* Ain);
	// calc needed x and y for the legs for a given gamma
	Result getXYforSpin(double gamma, double currX, double currZ, double& xReq13, double& xReq24, double& zReq13, double& zReq24);
	Matrix<double>& getA_R_W();
	double getCurrGamma() const;
	// dtor
	~COM();
protected:
	Matrix<double> A_R_W{4, 4}; // curr robot to world transformation
	Matrix<double> last_A_R_W{ 4, 4 };	
	double currGamma; // current orientation of the robot relative to the world in radian
	vector<double> COMxVec; 
	vector<double> COMyVec;
	vector<double> COMzVec;
	vector<double> gammaVec;
	LfLeg LfLeg_;
	RfLeg RfLeg_;
	RbLeg RbLeg_;
	LbLeg LbLeg_;
};

#endif // COM_H

