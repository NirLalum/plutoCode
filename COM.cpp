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

Result COM::initialRobot(){
    cout << "go initial position" << endl;
    Leg* legSequance[4] = { &LfLeg_, &RfLeg_, &LbLeg_, &RbLeg_ };
    vector<double> path;
    double InitHeight = 17;
    double FinHeight = yi; // zi = 30
    int steps = 20;
    linspace(path, InitHeight, FinHeight, steps); // build path
    #ifdef __linux__
	#define SERIAL_PORT "/dev/ttyACM0"
	#   endif
	cout << "start leg movement:" << endl << endl;
	serialib serialInit;
	char errorOpening = serialInit.openDevice(SERIAL_PORT, 115200);
	int Rate = 1000; // the rate of messages sending (for serial communication function)
    int i = 0;
    for(i = 0; i<path.size(); i++){
        // get thetas from xyz
        legSequance[0]->setInverseKinematics(-xi, path[i], zi);
        legSequance[1]->setInverseKinematics(xi, path[i], zi);
        legSequance[2]->setInverseKinematics(xi, path[i], -zi);
        legSequance[3]->setInverseKinematics(-xi, path[i], -zi);
        // send data to arduino
		LfLeg_.nextMove(serialInit, Rate);
		RfLeg_.nextMove(serialInit, Rate);
		RbLeg_.nextMove(serialInit, Rate);
		LbLeg_.nextMove(serialInit, Rate);
    }
    return SUCCESS;
}

// this function moves the robot forward and backward and updated each time the POSE of the robot
Result COM::robotBackwardForward(vector<double>& COMinitVec, vector<double>& COMfinVec, int forward) {

	Leg* legSequance[4] = { &LfLeg_, &RfLeg_, &LbLeg_, &RbLeg_ };
	if (forward == -1) {
		legSequance[0] = &LbLeg_; legSequance[1] = &RbLeg_; legSequance[2] = &LfLeg_; legSequance[3] = &RfLeg_;
	}

	double tiltDelta = 8; // define how much to tilt COM
	double legDelta = forwardDelta;
	vector<double> tiltVec{ tiltDelta, 0, 0, 1 }; // relative to the robot
	Matrix<double> tiltInRobotCords(4, 1, tiltVec); // relative to the robot
	Matrix<double> tiltInWorldCords = A_R_W * tiltInRobotCords; // transform to world cords

	// ---- start gait -------
	// tilt COM to the right // OK

	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	moveCOM(); // ------------------ not defined yet ---------------------
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	// move first leg
	legSequance[0]->legForwardBackWard(xi + tiltDelta, 0,  legDelta, forward);
	legSequance[0]->moveLeg();

	//bring back COM to initial pos // OK
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3], COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3]);
	// move next leg
	legSequance[1]->legForwardBackWard(xi, 0, legDelta, forward);
	legSequance[1]->moveLeg();
    // put delay here
    usleep(1000000);
	// push COM forward/backwawrds // OK
	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]); // update A_R_W
	// move next leg
	legSequance[2]->legForwardBackWard(xi, -legDelta, 0, forward);
	legSequance[2]->moveLeg();

	// tilt COM to the left
	tiltInRobotCords.setElement(1, 1, -tiltDelta);
	tiltInWorldCords = A_R_W * tiltInRobotCords;
	COMpathFunc(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]); // update A_R_W
	// move last leg
	legSequance[3]->legForwardBackWard(xi + tiltDelta, -legDelta, 0, forward);
	legSequance[3]->moveLeg();

	// tilt COM back to the center
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3], COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]); // update A_R_W

	// --- end of gait ----

	return SUCCESS;
}

Result COM::robotCrabWalk(vector<double>& COMinitVec, vector<double>& COMfinVec, int side) {
	Leg* legSequance[4] = { &RfLeg_, &RbLeg_, &LfLeg_, &LbLeg_ };
	if (side == -1) {
		legSequance[0] = &LfLeg_; legSequance[1] = &LbLeg_; legSequance[2] = &RfLeg_; legSequance[3] = &RbLeg_;
	}
	double tiltDelta = 8; // define how much to tilt COM
	double legDelta = crabDelta;
	vector<double> tiltVec{ -side * tiltDelta, 0, 0, 1 };
	Matrix<double> tiltInRobotCords(4, 1, tiltVec);
	Matrix<double> tiltInWorldCords = A_R_W * tiltInRobotCords;

	// ---- start gait -------
	// tilt COM to the right/left (depends on which side the crab walk is)

	COMpathFunc(COMinitVec[0], COMinitVec[1], COMinitVec[2], COMinitVec[3], tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3]);
	// move first leg
	legSequance[0]->legRightLeft(0, xi + tiltDelta, xi + tiltDelta + legDelta, 1);
	legSequance[0]->moveLeg();

	// move next leg
	legSequance[1]->legRightLeft(0, xi + tiltDelta, xi + tiltDelta + legDelta, 1);
	legSequance[1]->moveLeg();

	// push COM to the other side
	tiltInRobotCords.setElement(1, 1, side * (legDelta + 2 * tiltDelta));
	Matrix<double> tiltInWorldCords2 = A_R_W * tiltInRobotCords;
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), COMfinVec[3], tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), COMfinVec[3]); // update A_R_W
	// move next leg
	legSequance[2]->legRightLeft(0, xi + tiltDelta + legDelta, xi + tiltDelta, 1);
	legSequance[2]->moveLeg();

	// move last leg

	legSequance[3]->legRightLeft(0, xi + tiltDelta + legDelta, xi + tiltDelta, 1);
	legSequance[3]->moveLeg();

	// move COM to desired final location
	COMpathFunc(tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), COMfinVec[3], COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]);
	moveCOM();
	setRobotToWorldTrans(COMfinVec[0], COMfinVec[1], COMfinVec[2], COMfinVec[3]); // update A_R_W

	// --- end of gait ----

	return SUCCESS;
}

// the gait that makes the robot turn
Result COM::robotTurn(double gammaInit, double gammaFin, double currX, double currY, double currZ) {
	Leg* legSequance[4] = { &LfLeg_, &LbLeg_, &RbLeg_, &RfLeg_ };
	int aux1 = 1, aux2 = -1;
	if (gammaFin < gammaInit) {
		 aux1 = -1, aux2 = 1;
	}

	double tiltDelta = 8; // define how much to tilt COM
	double legDelta = forwardDelta;
	vector<double> tiltVec{ tiltDelta, 0, 0, 1 };
	Matrix<double> tiltInRobotCords(4, 1, tiltVec);
	Matrix<double> tiltInWorldCords = A_R_W * tiltInRobotCords;
	double xReq13, xReq24, zReq13, zReq24;
	// calc the required x and z for each leg (legs 1 and 3 have the x and z also 2 and 4)
	getXYforSpin(gammaFin - gammaInit, xi, zi, xReq13, xReq24, zReq13, zReq24);

	// ---- start gait -------
	// tilt COM to the right

	COMpathFunc(currX, currY, currZ, gammaInit, tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), gammaInit);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), gammaInit);
	// move first leg
	legSequance[0]->legTurn(xi + tiltDelta, aux1*xReq13 + tiltDelta, 0, aux2*zReq13);
	legSequance[0]->moveLeg();

	// move next leg
	legSequance[1]->legTurn(xi + tiltDelta, aux2*xReq24 + tiltDelta, 0, aux2*zReq24);
	legSequance[1]->moveLeg();

	// tilt COM to the left
	tiltInRobotCords.setElement(1, 1, -2 * tiltDelta);
	Matrix<double> tiltInWorldCords2 = A_R_W * tiltInRobotCords;
	COMpathFunc(tiltInWorldCords.getElement(1, 1), tiltInWorldCords.getElement(2, 1), tiltInWorldCords.getElement(3, 1), gammaInit, tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), gammaInit);
	moveCOM();
	setRobotToWorldTrans(tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), gammaInit); // update A_R_W
	// move next leg
	legSequance[2]->legTurn(xi + tiltDelta, aux1*xReq13 + tiltDelta, 0, aux1*zReq13);
	legSequance[2]->moveLeg();

	// move last leg
	legSequance[3]->legTurn(xi + tiltDelta, aux2*xReq24 + tiltDelta, 0, aux1*zReq24);
	legSequance[3]->moveLeg();

	// tilt COM back to the center
	COMpathFunc(tiltInWorldCords2.getElement(1, 1), tiltInWorldCords2.getElement(2, 1), tiltInWorldCords2.getElement(3, 1), gammaInit, currX, currY, currZ, gammaInit);
	moveCOM();
	setRobotToWorldTrans(currX, currY, currZ, gammaInit); // update A_R_W

	// spin COM cw/ccw around itself
	COMpathFunc(currX, currY, currZ, gammaInit, currX, currY, currZ, gammaFin);
	moveCOM();
	setRobotToWorldTrans(currX, currY, currZ, gammaFin); // update A_R_W
	currGamma = gammaFin; // update robot orientation

	// --- end of gait ----

	return SUCCESS;




	return SUCCESS;
}


// this function moves the center of mass of the robot
Result COM::moveCOM() {
	cout << endl << "start com movement:" << endl << endl;
	#ifdef __linux__
	#define SERIAL_PORT "/dev/ttyACM0"
	#   endif
	cout << "start leg movement:" << endl << endl;
	int i = 1;
	int Rate = 1000; // for serial comunication function
	serialib serialCOM;
	char errorOpening = serialCOM.openDevice(SERIAL_PORT, 115200);
	for (i; i < COMxVec.size(); i++) {
		setParallelInverseKinematics(COMxVec[i], COMyVec[i], COMzVec[i], gammaVec[i]);
		// send desired thetas to arduino
		LfLeg_.nextMove(serialCOM, Rate);
		RfLeg_.nextMove(serialCOM, Rate);
		RbLeg_.nextMove(serialCOM, Rate);
		LbLeg_.nextMove(serialCOM, Rate);
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
	cout << "RbLeg thetas:" << endl << RbLeg_.getTheta1() << endl << RbLeg_.getTheta2() << endl << RbLeg_.getTheta3() << endl;
	calcOneLegParallelInvesrKinematics(&LbLeg_, initializeVec4, pxDes, pyDes, pzDes, gammaDes);
	cout << "LbLeg thetas:" << endl << LbLeg_.getTheta1() << endl << LbLeg_.getTheta2() << endl << LbLeg_.getTheta3() << endl << endl;
	return SUCCESS;
}

Result COM::COMpathFunc(double cmXinit, double cmYinit, double cmZinit, double gammaInit, double cmXfin, double cmYfin, double cmZfin, double gammaFin) {
	int stepsNum = 20;
	int i = 0;
	// deleting previous pathes
	COMxVec.erase(COMxVec.begin(), COMxVec.end());
	COMyVec.erase(COMyVec.begin(), COMyVec.end());
	COMzVec.erase(COMzVec.begin(), COMzVec.end());
	gammaVec.erase(gammaVec.begin(), gammaVec.end());
	// start path plan
	linspace(COMxVec, cmXinit, cmXfin, stepsNum);
	linspace(COMyVec, cmYinit, cmYfin, stepsNum);
	linspace(COMzVec, cmZinit, cmZfin, stepsNum);
	linspace(gammaVec, gammaInit, gammaFin, stepsNum);
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
	// update leg current pos (not real values for now)
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

Result COM::getXYforSpin(double gamma, double currX, double currZ, double& xReq13, double& xReq24, double& zReq13, double& zReq24) {
	double length = 0, width = 0, a1 = 0, b1 = 0, c1 = 0, a2 = 0, b2 = 0, c2 = 0, x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;
	double gammaAux = abs(gamma); // in case gamma is negative because the robot turns cw
	length = H + 2 * abs(currZ);
	width = w + 2 * abs(currX);

	b1 = atan2(width / 2, length / 2);
	c1 = (PI - gammaAux) / 2;
	a1 = c1 - b1;
	z1 = sqrt(pow(length, 2) + pow(width, 2)) * sin(gammaAux / 2);
	y1 = z1 * cos(a1);
	x1 = z1 * sin(a1) + currX;

	b2 = atan2(length / 2, width / 2);
	c2 = (PI - gammaAux) / 2;
	a2 = c2 - b2;
	z2 = sqrt(pow(length, 2) + pow(width, 2)) * sin(gammaAux / 2);
	x2 = z2 * cos(a2) - currX; // lengthowidth muclength to spin in x
	y2 = z2 * sin(a2);

	if (gamma > 0) {
		xReq13 = x1;
		zReq13 = y1;
		xReq24 = x2;
		zReq24 = y2;
	}
	else // if the robot turns in cw direction
	{
		xReq13 = x2;
		zReq13 = y2;
		xReq24 = x1;
		zReq24 = y1;
	}
	return SUCCESS;
}

Matrix<double>& COM::getA_R_W(){
	return A_R_W;
}

double COM::getCurrGamma() const {
	return currGamma;
}

COM::~COM() {}
