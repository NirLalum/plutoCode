// right backward leg definition (leg 3 in matlab)

#ifndef RBLEG_H
#define RBLEG_H

#include "Leg.H"

class RbLeg : public Leg {
public:
	RbLeg(); // if transform is needed the matrix will be build here
	Result setInverseKinematics(double x, double y, double z);
	// for forward (forward = 1) and backward (forward = -1) movement
	Result legForwardBackWard(double xInit, double zInit, double zFin, int forward);
	// for crabWalk:
	// 1 for right, -1 for left
	Result legRightLeft(double zInit, double xInit, double xFin, int side);
	void print(ostream& co);
	~RbLeg();
protected:
	//Mat RfLeg2RobotTrans; // not sure if needed
};

#endif

