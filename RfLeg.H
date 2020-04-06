// right forward leg definition

// right forward leg (leg 2 in matlab)

#ifndef RFLEG_H
#define RFLEG_H

#include "Leg.H"

/*ostream& operator<<(ostream& co, Leg& leg) {
	leg.print(co);
	return co;
} */

class RfLeg : public Leg {
public:
	RfLeg(); // if transform is needed the matrix will be build here
	Result setInverseKinematics(double x, double y, double z);
	void print(ostream& co);
	~RfLeg(); 
protected:

};

#endif