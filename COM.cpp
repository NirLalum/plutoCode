#include "COM.H"

COM::COM() : currGamma(0) {
	vector<double> initializeVector{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	try
	{
		A_R_W.MatrixFromVector(initializeVector);
	}
	catch (exception & Error)
	{
		cout << Error.what() << endl;
	}
	//A_R_W.printMatrix();
}

Result COM::setParallelInvesrKinematics(double px, double py, double pz, double desGamma) {
	return SUCCES;
}

Result COM::getRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma) {
	vector<double> valuesVec{ cos(gamma), -sin(gamma), 0, cmX,
							sin(gamma), cos(gamma), 0, cmY,
							0, 0, 1, cmZ,
							0, 0, 0, 1 };
	try
	{
		A_R_W.MatrixFromVector(valuesVec);
	}
	catch (exception& Error)
	{
		cout << Error.what() << endl;
		return FAIL;
	}
	return SUCCES;
}

COM::~COM() {}