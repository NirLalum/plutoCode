#include "COM.H"

COM::COM() : currGamma(0) {
	vector<double> initializeVector{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	try
	{
		A_R_W = initializeVector;
	}
	catch (exception & Error)
	{
		cout << "error in COM c'tor: " << Error.what() << endl;
	}
	//A_R_W.printMatrix();
}

// this function moves the robot forward and backward and updated each time the POSE of the robot
Result COM::robotBackwardForward(vector<double>& COMinitVec, vector<double>& COMfinVec, int forward) {
	Leg* legSequance[4] = { &LfLeg_, &RfLeg_, &LbLeg_, &RbLeg_ };
	if (forward == -1) Leg* legSequance[4] = { &LbLeg_, &RbLeg_, &LfLeg_, &RfLeg_ };
	double tiltDelta = 8; // define how much to tilt COM 
	// tilt COM to the right
	vector<double> tiltInRobotCords{ tiltDelta, 0, 0, 1 };
	Matrix<double> tiltInWorldCords = A_R_W * tiltInRobotCords; // -------- finish definition of operator * for stl vector
	tiltInWorldCords.setElement(4,1,COMfinVec[3]);
	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), tiltInWorldCords.getElement(4, 1));
	moveCOM(); // not defined yet
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), tiltInWorldCords.getElement(4, 1));
	




	return SUCCESS;
}


Result COM::COMpathFunc(double cmXinit, double cmYinit, double cmZinit, double gammaInit, double cmXfin, double cmYfin, double cmZfin, double gammaFin) {
	int stepsNum = 10;
	int i = 0;
	COMxVec[i] = 0;
	COMyVec[i] = 0;
	COMzVec[i] = 0;
	gammaVec[i] = 0;
	for (i = 1; i <= stepsNum; i++) {
		COMxVec[i] = COMxVec[i - 1] + (cmXfin - cmXinit) / stepsNum;
		COMyVec[i] = COMyVec[i - 1] + (cmYfin - cmYinit) / stepsNum;
		COMzVec[i] = COMzVec[i - 1] + (cmZfin - cmZinit) / stepsNum;
		gammaVec[i] = gammaVec[i - 1] + (gammaFin - gammaInit) / stepsNum;
	}
	return SUCCESS;
}


Result COM::setParallelInverseKinematics(double pxDes, double pyDes, double pzDes, double gammaDes) {
	// ---------- add here work space condition ------------------
	vector<double> initializeVec1{ -w / 2, H / 2, 0, 1 };
	vector<double> initializeVec2{ w / 2, H / 2, 0, 1 };
	vector<double> initializeVec3{ w / 2, -H / 2, 0, 1 };
	vector<double> initializeVec4{ -w / 2, -H / 2, 0, 1 };
	calcOneLegParallelInvesrKinematics(&LfLeg_, initializeVec1, pxDes, pyDes, pzDes, gammaDes);
	calcOneLegParallelInvesrKinematics(&RfLeg_, initializeVec2, pxDes, pyDes, pzDes, gammaDes);
	calcOneLegParallelInvesrKinematics(&RbLeg_, initializeVec3, pxDes, pyDes, pzDes, gammaDes);
	calcOneLegParallelInvesrKinematics(&LbLeg_, initializeVec4, pxDes, pyDes, pzDes, gammaDes);

	return SUCCESS;
}

// this func calcualets the parallel robot configuration inverse kinematics
Result COM::calcOneLegParallelInvesrKinematics(Leg* currLeg, vector<double>& CMtoShValues,double pxDes, double pyDes, double pzDes, double gammaDes) {
	
	//define vectors from com to the leg shoulder
	Matrix<double>  CMtoSh(4, 1, CMtoShValues);
	
	// update robot to world transformation matrix
	setRobotToWorldTrans(pxDes, pyDes, pzDes, gammaDes);
	// define each foot location relative to the world
	Matrix<double> footWorldLocation = A_R_W * currLeg->getA_leg_R() * currLeg->getCurrentLocation();
	// calculating the vector from the shoulder to the foot for each leg and then setting the invesrse kinematics for each leg
	Matrix<double> tempMat = A_R_W * CMtoSh;
	Matrix<double> S = footWorldLocation - tempMat;
	// calc world to leg transform
	Matrix<double> invAux = A_R_W * currLeg->getA_leg_R();
	Matrix<double> A_leg_W = getAinv(invAux);
	// calc S vector relative ti the world
	Matrix<double> SlicedMat1 = A_leg_W.sliceMatrix(1, 3, 1, 3);
	Matrix<double> SlicedMat2 = S.sliceMatrix(1, 3, 1, 1);
	S = SlicedMat1 * SlicedMat2;
	// setting the desired angles for each leg motors
	// ------------------- add here some try and catch for leg workspace ----------------------------
	currLeg->setInverseKinematics(S.getElement(1, 1), S.getElement(2, 1), S.getElement(3, 1));
	return SUCCESS;
}

Matrix<double> COM::getAinv(Matrix<double>& Ain) {
	Matrix<double> Aout(4, 4);
	Matrix<double> auxVec(3, 1);
	Matrix<double> auxMat(3, 3);
	vector<double> zerosVector{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	Matrix<double> zerosMat(3, 3, zerosVector);
	auxMat = Ain.sliceMatrix(1, 3, 1, 3);
	auxVec = Ain.sliceMatrix(1, 3, 4, 4);
	Aout = auxMat.getTranspose(); // defining rotation part of A out
	auxVec = (zerosMat - auxMat).getTranspose() * auxVec;
	int i = 1, j = 4;
	for (i = 1; i <= 3; i++) {
		Aout.setElement(i, j, auxVec.getElement(i, 1));
	}
	for (i = 4, j = 1; j <= 4; j++) {
		Aout.setElement(i, j, j == 4 ? 1 : 0);
	}
	return Aout;
}

Result COM::setRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma) {
	vector<double> valuesVec{ cos(gamma), -sin(gamma), 0, cmX,
							sin(gamma), cos(gamma), 0, cmY,
							0, 0, 1, cmZ,
							0, 0, 0, 1 };
	try
	{
		A_R_W = valuesVec;
	}
	catch (exception& Error)
	{
		cout << Error.what() << endl;
		return FAIL;
	}
	return SUCCESS;
}

COM::~COM() {}