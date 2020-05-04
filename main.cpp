#include <iostream>
#include <string>
#include <stdlib.h>

#include "COM.H"
#include "RfLeg.H"
#include "LfLeg.H"

using namespace std;

int main() {
	COM robotCOM;
	RbLeg leg;
//	leg.pathFunc(0, -30);
	double ForwardDelta = forwardDelta;
	double CrabDelta = crabDelta;
	double SpinDelta = spinDelta;
	Matrix<double> CurrComPos(4, 1); // relative to the world
	Matrix<double> DesComPos(4, 1); // relative to the world
	Matrix<double> DesComPosRelativeToRobot(4, 1);
	vector<double> InitVec;
	vector<double> FinVec;
	char KeyBoardInput;
	cout << "where to go?" << endl;
	cin >> KeyBoardInput;
	while (KeyBoardInput != 't') {
		switch (KeyBoardInput) {
		// move forward
		case 'w': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4); // calc current location relative to the world
					// desired location relative to the robot
					DesComPosRelativeToRobot.setElement(1, 1, 0); DesComPosRelativeToRobot.setElement(2, 1, ForwardDelta); DesComPosRelativeToRobot.setElement(3, 1, 0); DesComPosRelativeToRobot.setElement(4, 1, 1);
					DesComPos = robotCOM.getA_R_W() * DesComPosRelativeToRobot;// calc desired location relative to the world
					vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma()}; 
					vector<double> VectorFin{ DesComPos.getElement(1,1), DesComPos.getElement(2,1), DesComPos.getElement(3,1), robotCOM.getCurrGamma()};
					robotCOM.robotBackwardForward(VectorInit, VectorFin, 1); // move robot
					break;
		}
		// move backwards
		case 's': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4);
					DesComPosRelativeToRobot.setElement(1, 1, 0); DesComPosRelativeToRobot.setElement(2, 1, -ForwardDelta); DesComPosRelativeToRobot.setElement(3, 1, 0); DesComPosRelativeToRobot.setElement(4, 1, 1);
					DesComPos = robotCOM.getA_R_W() * DesComPosRelativeToRobot; // relative to the world
					vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma() };
					vector<double> VectorFin{ DesComPos.getElement(1,1), DesComPos.getElement(2,1), DesComPos.getElement(3,1), robotCOM.getCurrGamma() };
					robotCOM.robotBackwardForward(VectorInit, VectorFin, -1);
					break;
		}
		// crab walk to the right
		case 'e': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4);
					DesComPosRelativeToRobot.setElement(1, 1, CrabDelta); DesComPosRelativeToRobot.setElement(2, 1, 0); DesComPosRelativeToRobot.setElement(3, 1, 0); DesComPosRelativeToRobot.setElement(4, 1, 1);
					DesComPos = robotCOM.getA_R_W() * DesComPosRelativeToRobot; // relative to the world
					vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma() };
					vector<double> VectorFin{ DesComPos.getElement(1,1), DesComPos.getElement(2,1), DesComPos.getElement(3,1), robotCOM.getCurrGamma() };
					robotCOM.robotCrabWalk(VectorInit, VectorFin, 1);
					break;
		}
		// crab walk to the left
		case 'q': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4);
					DesComPosRelativeToRobot.setElement(1, 1, -CrabDelta); DesComPosRelativeToRobot.setElement(2, 1, 0); DesComPosRelativeToRobot.setElement(3, 1, 0); DesComPosRelativeToRobot.setElement(4, 1, 1);
					DesComPos = robotCOM.getA_R_W() * DesComPosRelativeToRobot; // relative to the world
					vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma() };
					vector<double> VectorFin{ DesComPos.getElement(1,1), DesComPos.getElement(2,1), DesComPos.getElement(3,1), robotCOM.getCurrGamma() };
					robotCOM.robotCrabWalk(VectorInit, VectorFin, -1);
					break;
		}
		// spin robot CW
		case 'd': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4);
					vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma() };
					robotCOM.robotTurn(robotCOM.getCurrGamma(), robotCOM.getCurrGamma() - SpinDelta, VectorInit[0], VectorInit[1], VectorInit[2]);
					break;
		}
		// spin robot CCW
		case 'a': { CurrComPos = robotCOM.getA_R_W().sliceMatrix(1, 4, 4, 4);
				    vector<double> VectorInit{ CurrComPos.getElement(1,1), CurrComPos.getElement(2,1), CurrComPos.getElement(3,1), robotCOM.getCurrGamma() };
					robotCOM.robotTurn(robotCOM.getCurrGamma(), robotCOM.getCurrGamma() + SpinDelta, VectorInit[0], VectorInit[1], VectorInit[2]);
					break;
		}
		}
		cout << "where to go?" << endl;
		cin >> KeyBoardInput;
	}
	return 0;
}	

