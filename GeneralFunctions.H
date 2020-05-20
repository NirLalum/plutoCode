
#ifndef GeneralFunctions_H
#define GeneralFunctions_H

#include <vector>
#include "Definitions.H"
#include <string>
//#include <bits>
// Serial library
#include "serialib.H"
#include <unistd.h>
#include <stdio.h>


using namespace std;

Result linspace(vector<double>& Vec, double InitVal, double FinVal ,int stepsNum);
// sending data and retrieving data (raspberypi <-> arduino)
Result serialComunication(vector<double> thetasDouble, serialib& serial);

#endif // !GeneralFunctions_H

