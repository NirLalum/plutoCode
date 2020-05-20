// abstract class of a leg

#include <math.h>
#include <vector>
#include "Definitions.H"
#include "GenericMatrix.H"
#include "GeneralFunctions.H"
#include <iostream>
#include "serialib.H"

using namespace std;

#ifndef Leg_H_
#define Leg_H_

// define consts for initial position of each leg (relative to the shoulder)
# define xi l2
# define yi 30
# define zi 0


class Leg {
public:
	// Leg constructor
	Leg();
	double getTheta1() const;
	double getTheta2() const;
	double getTheta3() const;
	virtual Result setInverseKinematics(double px, double py, double pz) = 0;
	// for forward (forward = 1) and backward (forward = -1) movement
	virtual Result legForwardBackWard(double xInit, double zInit, double zFin, int forward);
	// for crabWalk:
	// 1 for right, -1 for left
	virtual Result legRightLeft(double zInit, double xInit, double xFin, int side);
	// creates a path for the leg to make to robot turn around itself
	virtual Result legTurn(double xInit, double xFin, double zInit, double zFin);
	// this func wiil calc the leg path (x, y, z) from given conditions
	Result pathFunc(double z0, double zf);
	// move a leg according to a pre made path (only when one leg in moving' not in parallel moving)
	Result moveLeg();
	// moves leg according to pre made pathes (convert xyz to thetas)
	int nextMove(serialib& serial);
	virtual void print(ostream& co) = 0;
	// get leg to robot transformation matrix
	Matrix<double>& getA_leg_R();
	// get current location of the leg foot relative to the leg shoulder
	Matrix<double>& getCurrentLocation();
	void setCurrentLocation(double x, double y, double z);
	// Leg Detor
	virtual ~Leg();
protected:
	double theta1_;
	double theta2_;
	double theta3_;
	vector <double> xVec;
	vector <double> yVec;
	vector <double> zVec;
	Matrix <double> A_leg_R{ 4, 4 };
	Matrix <double> CurrLocation{ 4, 1 }; // relative to the robot shoulder axis (update in COM gait)
};


#endif

