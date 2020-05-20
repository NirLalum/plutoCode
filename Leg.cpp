#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include "Leg.H"
#include "Definitions.H"
#include "GeneralFunctions.H"

Leg::Leg() : theta1_(theta1Init), theta2_(theta2Init), theta3_(theta3Init) {}

double Leg::getTheta1() const {
	return theta1_;
}

double Leg::getTheta2() const {
	return theta2_;
}

double Leg::getTheta3() const {
	return theta3_;
}

Matrix<double>& Leg::getA_leg_R() {
	return A_leg_R;
}

Matrix<double>& Leg::getCurrentLocation() {
	return CurrLocation;
}

// updating current location of the leg (x y z relative to the shoulder axis)
void Leg::setCurrentLocation(double x, double y, double z) {
	CurrLocation.setElement(1, 1, x);
	CurrLocation.setElement(2, 1, y);
	CurrLocation.setElement(3, 1, z);
}

// this function creates a yz path for a leg
Result Leg::pathFunc(double z0, double zf) {
	// a b c d e f g h i are the input and output (z/x and y) poly coefs (and their symbolic value was calculated in matlab)
	int flag = 0;
	if (abs(z0) > abs(zf)) {
		double tmp = z0;
		z0 = zf;
		zf = tmp;
		flag = 1;
	}
	double a, b, c, d, e, f, g, h, i, delta, t0, y0, yf, yMax, zMax, tf, v0, vf, a0, af, inValue, outValue;
	delta = 0.01; // determines the time step
	t0 = 0;
	tf = 0.1;
	y0 = 30;
	yf = 30;
	yMax = 25;
	zMax = (zf + z0) / 2;
	v0 = 0;
	vf = 0;
	a0 = 0;
	af = 0;
	// input polycoefs:
	a = (12 * z0 - 12 * zf - 6 * t0 * v0 - 6 * t0 * vf + 6 * tf * v0 + 6 * tf * vf + a0 * pow(t0, 2) + a0 * pow(tf,2) - af * pow(t0,2) - af * pow(tf,2) - 2 * a0 * t0 * tf + 2 * af * t0 * tf) / (2 * pow(t0 - tf , 5));
	b = -(30 * t0 * z0 - 30 * t0 * zf + 30 * tf * z0 - 30 * tf * zf + 2 * a0 * pow(t0, 3) + 3 * a0 * pow(tf, 3) - 3 * af * pow(t0, 3) - 2 * af * pow(tf, 3) - 14 * pow(t0, 2) * v0 - 16 * pow(t0, 2) * vf + 16 * pow(tf, 2) * v0 + 14 * pow(tf, 2) * vf - 4 * a0 * t0 * pow(tf, 2) - a0 * pow(t0, 2) * tf + af * t0 * pow(tf, 2) + 4 * af * pow(t0, 2) * tf - 2 * t0 * tf * v0 + 2 * t0 * tf * vf) / (2 * pow((t0 - tf), 5));
	c = (a0 * pow(t0, 4) + 3 * a0 * pow(tf, 4) - 3 * af * pow(t0, 4) - af * pow(tf, 4) - 8 * pow(t0, 3) * v0 - 12 * pow(t0, 3) * vf + 12 * pow(tf, 3) * v0 + 8 * pow(tf, 3) * vf + 20 * pow(t0, 2) * z0 - 20 * pow(t0, 2) * zf + 20 * pow(tf, 2) * z0 - 20 * pow(tf, 2) * zf + 4 * a0 * pow(t0, 3) * tf - 4 * af * t0 * pow(tf, 3) + 28 * t0 * pow(tf, 2) * v0 - 32 * pow(t0, 2) * tf * v0 + 32 * t0 * pow(tf, 2) * vf - 28 * pow(t0, 2) * tf * vf - 8 * a0 * pow(t0, 2) * pow(tf, 2) + 8 * af * pow(t0, 2) * pow(tf, 2) + 80 * t0 * tf * z0 - 80 * t0 * tf * zf) / (2 * pow((t0 - tf), 5));
	d = -(a0 * pow(tf, 5) - af * pow(t0, 5) + 4 * a0 * t0 * pow(tf, 4) + 3 * a0 * pow(t0, 4) * tf - 3 * af * t0 * pow(tf, 4) - 4 * af * pow(t0, 4) * tf + 36 * t0 * pow(tf, 3) * v0 - 24 * pow(t0, 3) * tf * v0 + 24 * t0 * pow(tf, 3) * vf - 36 * pow(t0, 3) * tf * vf + 60 * t0 * pow(tf, 2) * z0 + 60 * pow(t0, 2) * tf * z0 - 60 * t0 * pow(tf, 2) * zf - 60 * pow(t0, 2) * tf * zf - 8 * a0 * pow(t0, 2) * pow(tf, 3) + 8 * af * pow(t0, 3) * pow(tf, 2) - 12 * pow(t0, 2) * pow(tf, 2) * v0 + 12 * pow(t0, 2) * pow(tf, 2) * vf) / (2 * pow((t0 - tf), 5));
	e = (2 * pow(t0, 5) * vf - 2 * pow(tf, 5) * v0 + 2 * a0 * t0 * pow(tf, 5) - 2 * af * pow(t0, 5) * tf + 10 * t0 * pow(tf, 4) * v0 - 10 * pow(t0, 4) * tf * vf - a0 * pow(t0, 2) * pow(tf, 4) - 4 * a0 * pow(t0, 3) * pow(tf, 3) + 3 * a0 * pow(t0, 4) * pow(tf, 2) - 3 * af * pow(t0, 2) * pow(tf, 4) + 4 * af * pow(t0, 3) * pow(tf, 3) + af * pow(t0, 4) * pow(tf, 2) + 16 * pow(t0, 2) * pow(tf, 3) * v0 - 24 * pow(t0, 3) * pow(tf, 2) * v0 + 24 * pow(t0, 2) * pow(tf, 3) * vf - 16 * pow(t0, 3) * pow(tf, 2) * vf + 60 * pow(t0, 2) * pow(tf, 2) * z0 - 60 * pow(t0, 2) * pow(tf, 2) * zf) / (2 * pow((t0 - tf), 5));
	f = (2 * pow(t0, 5) * zf - 2 * pow(tf, 5) * z0 + 2 * t0 * pow(tf, 5) * v0 - 2 * pow(t0, 5) * tf * vf + 10 * t0 * pow(tf, 4) * z0 - 10 * pow(t0, 4) * tf * zf - a0 * pow(t0, 2) * pow(tf, 5) + 2 * a0 * pow(t0, 3) * pow(tf, 4) - a0 * pow(t0, 4) * pow(tf, 3) + af * pow(t0, 3) * pow(tf, 4) - 2 * af * pow(t0, 4) * pow(tf, 3) + af * pow(t0, 5) * pow(tf, 2) - 10 * pow(t0, 2) * pow(tf, 4) * v0 + 8 * pow(t0, 3) * pow(tf, 3) * v0 - 8 * pow(t0, 3) * pow(tf, 3) * vf + 10 * pow(t0, 4) * pow(tf, 2) * vf - 20 * pow(t0, 2) * pow(tf, 3) * z0 + 20 * pow(t0, 3) * pow(tf, 2) * zf) / (2 * pow((t0 - tf), 5));
	// output polycoefs
	g = (y0 * zMax - yMax * z0 - y0 * zf + yf * z0 + yMax * zf - yf * zMax) / ((z0 - zf) * (z0 * zMax - z0 * zf + zMax * zf - pow(zMax, 2)));
	h = -(y0 * pow(zMax, 2) - yMax * pow(z0, 2) - y0 * pow(zf, 2) + yf * pow(z0, 2) + yMax * pow(zf, 2) - yf * pow(zMax, 2)) / ((z0 - zf) * (z0 * zMax - z0 * zf + zMax * zf - pow(zMax, 2)));
	double i2 = 0;
    i = (y0 * pow(zf, 2) + yf * pow(z0, 2) + 2 * z0 * zf * sqrt(-(y0 - yMax) * (yMax - yf)) - 2 * yMax * z0 * zf) / (pow(z0, 2) - 2 * z0 * zf + pow(zf, 2));
	i2 = (y0 * pow(zf, 2) + yf * pow(z0, 2) - 2 * z0 * zf * sqrt((y0 - yMax) * (yf - yMax)) - 2 * yMax * z0 * zf) / (pow(z0, 2) - 2 * z0 * zf + pow(zf, 2));

	for (double t = t0; t <= tf; t+= delta) {
		inValue = a * pow(t, 5) + b * pow(t, 4) + c * pow(t, 3) + d * pow(t, 2) + e * t + f;
		outValue = g * pow(inValue,2) + h * inValue + i;
		zVec.push_back(inValue);
		yVec.push_back(outValue);
	}
	if (flag == 1) {
		reverse(zVec.begin(), zVec.end());
		reverse(yVec.begin(), yVec.end());
	}
	return SUCCESS;
}

Result Leg::legForwardBackWard(double xInit, double zInit, double zFin, int forward) {
	pathFunc(forward*zInit, forward * zFin);
	int size = yVec.size();
	for (int i = 0; i < size; i++) {
		xVec.push_back(xInit);
	}
	return SUCCESS;
}


Result Leg::legRightLeft(double zInit, double xInit, double xFin, int side) {
	pathFunc(xInit, side * xFin);
	int size = yVec.size();
	xVec = zVec;
	zVec.erase(zVec.begin(), zVec.end());
	for (int i = 0; i < size; i++) {
		zVec.push_back(zInit);
	}
	return SUCCESS;
}

Result Leg::legTurn(double xInit, double xFin, double zInit, double zFin) {
	pathFunc(zInit, zFin);
	int stepsNum = yVec.size();
	linspace(xVec, xInit, xFin, stepsNum);
	return SUCCESS;
}

Result Leg::moveLeg() {
	#ifdef __linux__
    #define SERIAL_PORT "/dev/ttyACM0"
    #   endif
	int index = 0;
	cout << "start leg movement:" << endl << endl;
    serialib serial;
    char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1) throw "serial openning faild";
	usleep(10000000); // 10 seconds in micro seconds. gives us time to open the serial monitor.
	for (index; index < xVec.size(); index++) {

		setInverseKinematics(xVec[index], yVec[index], zVec[index]);
		//cout << getTheta1() << endl << getTheta2() << endl << getTheta3() << endl;
		// send inverse kinematics data to arduino ----------------------------
		nextMove(serial);
		// implement here some kind of control func
		int i = 1;
		// update current location of the leg ------------------------------ (mean while not real values)
		if (index == xVec.size() - 1) {
			CurrLocation.setElement(1, 1, xVec[index]);
			CurrLocation.setElement(2, 1, yVec[index]);
			CurrLocation.setElement(3, 1, zVec[index]);
		}
	}
	serial.closeDevice();
	// end of step => earse path vectors
	xVec.erase(xVec.begin(), xVec.end());
	yVec.erase(yVec.begin(), yVec.end());
	zVec.erase(zVec.begin(), zVec.end());
	return SUCCESS;
}

int Leg::nextMove(serialib& serial) {
	// send and retrieve data from arduino
	vector<double> sendData{ theta1_, theta2_, theta3_ };
	vector<double> retrievedData;
	try { serialComunication(sendData, serial);}
	catch (const char* error) { cout << error << endl;}
	///// ---------------- implement here logical control ----------------

}

Leg::~Leg(){}
