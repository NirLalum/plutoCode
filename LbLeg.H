// left backward leg definition (leg 4 in matlab)

#ifndef LBLEG_H
#define LBLEG_H

#include "Leg.H"

class LbLeg : public Leg {
public:
	LbLeg(); // if transform is needed the matrix will be build here
	Result setInverseKinematics(double x, double y, double z);
	// for forward (forward = 1) and backward (forward = -1) movement
	Result legForwardBackWard(double xInit, double zInit, double zFin, int forward);
	// for crabWalk:
	// 1 for right, -1 for left
	Result legRightLeft(double zInit, double xInit, double xFin, int side);
	Result legTurn(double xInit, double xFin, double zInit, double zFin);
	void print(ostream& co);
	~LbLeg();
protected:
	//Mat RfLeg2RobotTrans; // not sure if needed
};

#endif

