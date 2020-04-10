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


Result COM::COMpathFunc(vector<double>& COMinitVec, vector<double>& COMfinVec) {
	int stepsNum = 10;
	int i = 0;
	COMxVec[i] = 0;
	COMyVec[i] = 0;
	COMzVec[i] = 0;
	gammaVec[i] = 0;
	for (i = 1; i <= stepsNum; i++) {
		COMxVec[i] = COMxVec[i - 1] + (COMfinVec[0] - COMinitVec[0]) / stepsNum;
		COMyVec[i] = COMyVec[i - 1] + (COMfinVec[1] - COMinitVec[1]) / stepsNum;
		COMzVec[i] = COMzVec[i - 1] + (COMfinVec[2] - COMinitVec[2]) / stepsNum;
		gammaVec[i] = gammaVec[i - 1] + (COMfinVec[3] - COMinitVec[3]) / stepsNum;
	}
	return SUCCESS;
}


// this func calcualets the parallel robot configuration inverse kinematics
Result COM::setParallelInvesrKinematics(double pxDes, double pyDes, double pzDes, double gammaDes) {
	// ---------- add here work space condition ------------------
	//define vectors from com to the leg shoulder
	vector<double> initializeVec1{ -w / 2, H / 2, 0, 1 };
	vector<double> initializeVec2{ w / 2, H / 2, 0, 1 };
	vector<double> initializeVec3{ w / 2, -H / 2, 0, 1 };
	vector<double> initializeVec4{ -w / 2, -H / 2, 0, 1 };
	Matrix<double>  CMtoSh1(4, 1, initializeVec1);
	Matrix<double>  CMtoSh2(4, 1, initializeVec2);
	Matrix<double>  CMtoSh3(4, 1, initializeVec3);
	Matrix<double>  CMtoSh4(4, 1, initializeVec4);
	// get desired robot to world transformation matrix
	Matrix<double> A_R_W_des = getRobotToWorldTrans(pxDes, pyDes, pzDes, gammaDes);
	// define each foot location relative to the world
	Matrix<double> foot1WorldLocation = A_R_W_des * LfLeg.getA_leg_R() * LfLeg.getCurrentLocation();
	Matrix<double> foot2WorldLocation = A_R_W_des * RfLeg.getA_leg_R() * RfLeg.getCurrentLocation();
	Matrix<double> foot3WorldLocation = A_R_W_des * RbLeg.getA_leg_R() * RbLeg.getCurrentLocation();
	Matrix<double> foot4WorldLocation = A_R_W_des * LbLeg.getA_leg_R() * LbLeg.getCurrentLocation();
	// calculating the vector from the shoulder to the foot for each leg and then setting the invesrse kinematics for each leg
	Matrix<double> S1 = foot1WorldLocation - A_R_W_des * CMtoSh1;
	Matrix<double> S2 = foot2WorldLocation - A_R_W_des * CMtoSh2;
	Matrix<double> S3 = foot3WorldLocation - A_R_W_des * CMtoSh3;
	Matrix<double> S4 = foot4WorldLocation - A_R_W_des * CMtoSh4;
	// calc world to leg transform
	Matrix<double> A_leg_W1 = getAinv(A_R_W_des * LfLeg.getA_leg_R());
	Matrix<double> A_leg_W2 = getAinv(A_R_W_des * RfLeg.getA_leg_R());
	Matrix<double> A_leg_W3 = getAinv(A_R_W_des * RbLeg.getA_leg_R());
	Matrix<double> A_leg_W4 = getAinv(A_R_W_des * LbLeg.getA_leg_R());
	// calc S vector relative ti the world
	S1 = A_leg_W1.sliceMatrix(1, 3, 1, 3) * S1.sliceMatrix(1, 3, 1, 1);
	S2 = A_leg_W1.sliceMatrix(1, 3, 1, 3) * S2.sliceMatrix(1, 3, 1, 1);
	S3 = A_leg_W1.sliceMatrix(1, 3, 1, 3) * S3.sliceMatrix(1, 3, 1, 1);
	S4 = A_leg_W1.sliceMatrix(1, 3, 1, 3) * S4.sliceMatrix(1, 3, 1, 1);
	// setting the desired angles for each leg motors
	// ------------------- add here some try and catch for leg workspace ----------------------------
	LfLeg.setInverseKinematics(S1.getElement(1, 1), S1.getElement(2, 1), S1.getElement(3, 1));
	RfLeg.setInverseKinematics(S2.getElement(1, 1), S2.getElement(2, 1), S2.getElement(3, 1));
	RbLeg.setInverseKinematics(S3.getElement(1, 1), S3.getElement(2, 1), S3.getElement(3, 1));
	LbLeg.setInverseKinematics(S4.getElement(1, 1), S4.getElement(2, 1), S4.getElement(3, 1));

	return SUCCESS;
}

Matrix<double>& COM::getAinv(Matrix<double>& Ain) {
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

Result COM::getRobotToWorldTrans(double cmX, double cmY, double cmZ, double gamma) {
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