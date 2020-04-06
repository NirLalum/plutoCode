#include "LbLeg.H"

LbLeg::LbLeg() : Leg() { 
    vector<double> initializeVector{ -1, 0, 0, -w / 2, 0, 0, -1, H / 2, 0, -1, 0, 0, 0, 0, 0, 1 };
    try
    {
        A_leg_R.MatrixFromVector(initializeVector);
    }
    catch (exception & Error)
    {
        cout << Error.what() << endl;
    }
    
    A_leg_R.printMatrix();
}

Result LbLeg::setInverseKinematics(double px, double py, double pz) {
    // need to add here work space of the robot
    int ARM = 1, ELB = -1, aux = 1;
    double C1, C3, S1, S3, fi, temp, px_new, py_new;
    C1 = -(l2 * py - px * sqrt(-pow(l2, 2) + pow(px, 2) + pow(py, 2))) / (pow(px, 2) + pow(py, 2));
    S1 = ARM * sqrt(1 - pow(C1, 2));
    fi = 0;
    temp = ((pow(px, 2) + pow(py, 2) + pow(pz - l1, 2))) - pow(l2, 2) - pow(l3, 2) - pow(l4, 2);
    C3 = temp / (2 * double(l3) * double(l4)); S3 = ELB * sqrt(1 - pow(C3, 2));
    theta3_ = aux * atan2(S3, C3);

    theta1_ = aux * atan2(S1, C1);

    py_new = py + l2 * cos(theta1_);
    px_new = px - l2 * sin(theta1_);
    theta2_ = fi + aux * (atan2((pz - l1), sqrt(pow(px_new, 2) + pow(py_new, 2))) - atan2(l4 * S3, (l3 + l4 * C3)));
    return SUCCES;
}

Result LbLeg::legForwardBackWard(double xInit, double zInit, double zFin, int forward) {
    return Leg::legForwardBackWard(xInit, -zInit, -zFin, forward);
}

Result LbLeg::legRightLeft(double zInit, double xInit, double xFin, int side) {
    return Leg::legRightLeft(-zInit, xInit, xFin, side);
}

void LbLeg::print(ostream& co) {
    co << "theta1:" << getTheta1() << endl << "theta2:" << getTheta2() << endl << "theta3:" << getTheta3() << endl << endl;
}

LbLeg::~LbLeg() {}