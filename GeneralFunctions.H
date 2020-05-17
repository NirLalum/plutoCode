
#ifndef GeneralFunctions_H
#define GeneralFunctions_H

#include <vector>
#include "Definitions.H"
#include <string>
//#include <bits>
// Serial library
#include "serialib.h"
#include <unistd.h>
#include <stdio.h>


using namespace std;

Result linspace(vector<double>& Vec, double InitVal, double FinVal ,int stepsNum);
// sending data and retrieving data (raspberypi <-> arduino)
vector<double> serialComunication(vector<double> thetasDouble);

#endif // !GeneralFunctions_H

