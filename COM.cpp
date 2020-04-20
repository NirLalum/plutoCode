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
	double legDelta = forwardDelta;
	vector<double> tiltVec{ tiltDelta, 0, 0, 1 };
	Matrix<double> tiltInRobotCords(4, 1, tiltVec);
	Matrix<double> tiltInWorldCords = A_R_W * tiltInRobotCords;

	// ---- start gait -------
	// tilt COM to the right // OK
	 
	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	moveCOM(); // ------------------ not defined yet ---------------------
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	// move first leg
	legSequance[0]->legForwardBackWard(l2 + tiltDelta, 0, forward * legDelta, 1);
	legSequance[0]->moveLeg();

	//bring back COM to initial pos // OK
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3], COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3]);
	// move next leg
	legSequance[1]->legForwardBackWard(l2, 0, forward * legDelta, 1);
	legSequance[1]->moveLeg();

	// push COM forward/backwawrds // OK
	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]); // update A_R_W
	// move next leg
	legSequance[2]->legForwardBackWard(l2, -forward * legDelta, 0, 1);
	legSequance[2]->moveLeg();

	// tilt COM to the left
	tiltInRobotCords.setElement(1, 1, -tiltDelta);
	tiltInWorldCords = A_R_W * tiltInRobotCords;
	COMpathFunc(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]); // update A_R_W
	// move last leg
	legSequance[3]->legForwardBackWard(l2 + tiltDelta, -forward * legDelta, 0, 1);
	legSequance[3]->moveLeg();

	// tilt COM back to the center
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3], COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]); // update A_R_W

	// --- end of gait ----

	return SUCCESS;
}


// this function moves the center of mass of the robot
Result COM::moveCOM() {
	int i = 1;
	cout << endl << "start com movement:" << endl << endl;
	for (i; i < COMxVec.size(); i++) {
		setParallelInverseKinematics(COMxVec[i], COMyVec[i], COMzVec[i], gammaVec[i]);
		// send desired thetas to arduino
		LfLeg_.nextMove();
		RfLeg_.nextMove();
		RbLeg_.nextMove();
		LbLeg_.nextMove();
	}
	return SUCCESS;
}


Result COM::COMpathFunc(double cmXinit, double cmYinit, double cmZinit, double gammaInit, double cmXfin, double cmYfin, double cmZfin, double gammaFin) {
	int stepsNum = 10;
	int i = 0;
	// deleting previous pathes
	COMxVec.erase(COMxVec.begin(), COMxVec.end());
	COMyVec.erase(COMyVec.begin(), COMyVec.end());
	COMzVec.erase(COMzVec.begin(), COMzVec.end());
	gammaVec.erase(gammaVec.begin(), gammaVec.end());
	// start path plan
	COMxVec.push_back(cmXinit);
	COMyVec.push_back(cmYinit);
	COMzVec.push_back(cmZinit);
	gammaVec.push_back(0);
	for (i = 1; i < stepsNum; i++) {
		COMxVec.push_back(COMxVec[i - 1] + (cmXfin - cmXinit) / (stepsNum - 1));
		COMyVec.push_back(COMyVec[i - 1] + (cmYfin - cmYinit) / (stepsNum - 1));
		COMzVec.push_back(COMzVec[i - 1] + (cmZfin - cmZinit) / (stepsNum - 1));
		gammaVec.push_back(gammaVec[i - 1] + (gammaFin - gammaInit) / (stepsNum - 1));
	}
	return SUCCESS;
}



Result COM::setParallelInverseKinematics(double pxDes, double pyDes, double pzDes, double gammaDes) {
	// ---------- add here work space condition ------------------
	vector<double> initializeVec1{ -w / 2, H / 2, 0, 1 };
	vector<double> initializeVec2{ w / 2, H / 2, 0, 1 };
	vector<double> initializeVec3{ w / 2, -H / 2, 0, 1 };
	vector<double> initializeVec4{ -w / 2, -H / 2, 0, 1 };
	last_A_R_W = A_R_W;
	// calc desired thetas aaccording to parallel inverse kinematics of the robot for each leg
	calcOneLegParallelInvesrKinematics(&LfLeg_, initializeVec1, pxDes, pyDes, pzDes, gammaDes); 
	cout << "LfLeg thetas:" << endl << LfLeg_.getTheta1() << endl << LfLeg_.getTheta2() << endl << LfLeg_.getTheta3() << endl;
	calcOneLegParallelInvesrKinematics(&RfLeg_, initializeVec2, pxDes, pyDes, pzDes, gammaDes);
	cout << "RfLeg thetas:" << endl << RfLeg_.getTheta1() << endl << RfLeg_.getTheta2() << endl << RfLeg_.getTheta3() << endl;
	calcOneLegParallelInvesrKinematics(&RbLeg_, initializeVec3, pxDes, pyDes, pzDes, gammaDes);
	cout << "RbLeg thetas:" << endl  << RbLeg_.getTheta1() << endl << RbLeg_.getTheta2() << endl << RbLeg_.getTheta3() << endl;
	calcOneLegParallelInvesrKinematics(&LbLeg_, initializeVec4, pxDes, pyDes, pzDes, gammaDes);
	cout << "LbLeg thetas:" << endl << LbLeg_.getTheta1() << endl << LbLeg_.getTheta2() << endl << LbLeg_.getTheta3() << endl << endl;
	return SUCCESS;
}

// this func calcualets the parallel robot configuration inverse kinematics
Result COM::calcOneLegParallelInvesrKinematics(Leg* currLeg, vector<double>& CMtoShValues,double pxDes, double pyDes, double pzDes, double gammaDes) {
	
	//define vectors from com to the leg shoulder
	Matrix<double>  CMtoSh(4, 1, CMtoShValues);
	// define each foot location relative to the world
	Matrix<double> footWorldLocation = last_A_R_W * currLeg->getA_leg_R() * currLeg->getCurrentLocation();
	// update robot to world transformation matrix
	setRobotToWorldTrans(pxDes, pyDes, pzDes, gammaDes);
	// calculating the vector from the shoulder to the foot for each leg and then setting the invesrse kinematics for each leg
	Matrix<double> tempMat = A_R_W * CMtoSh;
	Matrix<double> S = footWorldLocation - tempMat;
	// calc world to leg transform
	Matrix<double> A_W_leg = A_R_W * currLeg->getA_leg_R();
	getAinv(&A_W_leg);
	// calc S vector relative to the leg
	Matrix<double> SlicedMat1 = A_W_leg.sliceMatrix(1, 3, 1, 3);
	Matrix<double> SlicedMat2 = S.sliceMatrix(1, 3, 1, 1);
	S = SlicedMat1 * SlicedMat2;
	// setting the desired angles for each leg motors
	// ------------------- add here some try and catch for leg workspace ----------------------------
	currLeg->setInverseKinematics(S.getElement(1, 1), S.getElement(2, 1), S.getElement(3, 1));
	// update leg current pos
	currLeg->setCurrentLocation(S.getElement(1, 1), S.getElement(2, 1), S.getElement(3, 1));
	return SUCCESS;
}

Result COM::getAinv(Matrix<double>* Ain) {
	Matrix<double> auxVec(3, 1);
	Matrix<double> auxMat(4, 4);
	vector<double> zerosVector{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	Matrix<double> zerosMat(3, 3, zerosVector);
	auxMat = Ain->sliceMatrix(1, 3, 1, 3);
	auxVec = Ain->sliceMatrix(1, 3, 4, 4);
	*Ain = Ain->getTranspose(); // defining rotation part of A out
	auxVec = (zerosMat - auxMat).getTranspose() * auxVec;
	int i = 1, j = 4;
	for (i = 1; i <= 3; i++) {
		Ain->setElement(i, j, auxVec.getElement(i, 1));
	}
	for (i = 4, j = 1; j <= 4; j++) {
		Ain->setElement(i, j, j == 4 ? 1 : 0);
	}
	return SUCCESS;
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