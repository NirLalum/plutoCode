#include <iostream>
#include <string>
#include <stdlib.h>
#include "RfLeg.H"

using namespace std;

int main() {

    RfLeg leg;
	//vector<double> sendVec{1.2, 4.65, 3.212};
	//vector<double> backVec;
	//backVec = serialComunication(sendVec);
	leg.legForwardBackWard(xi, 0, 30, 1);
	leg.moveLeg();
}
