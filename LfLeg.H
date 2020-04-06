// left forward leg definition (leg 1 in matlab)

#ifndef LFLEG_H
#define LFLEG_H

#include "Leg.H"

class LfLeg : public Leg {
public:
	LfLeg(); // if transform is needed the matrix will be build here
	Result setInverseKinematics(double x, double y, double z);
	// for forward (forward = 1) and backward (forward = -1) movement
	Result legForwardBackWard(double xInit, double zInit, double zFin, int forward);
	// for crabWalk:
	// 1 for right, -1 for left
	Result legRightLeft(double zInit, double xInit, double xFin, int side);
	void print(ostream& co);
	~LfLeg();
protected:
	
};

#endif
