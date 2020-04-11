#include <iostream>
#include <string>
#include <stdlib.h>

#include "COM.H"
#include "RfLeg.H"
#include "LfLeg.H"

using namespace std;

int main() {
	//RfLeg leg1;
	COM com;
	//Leg* leg = &leg1;


	com.setParallelInverseKinematics(1, 0, 0, 0.1);

	//leg1.legForwardBackWard(l2, 0, 30, 1);

	//while (leg1.nextMove()) {
		//cout << leg1;
		//cout << "theta1:" << leg1.getTheta1() << endl << "theta2:" << leg1.getTheta2() << endl << "theta3:" << leg1.getTheta3() << endl << endl;
	//}
}	

