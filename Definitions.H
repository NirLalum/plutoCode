
#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define PI 3.1415926535897

// linkages size defintion (cm) 
#define l1  6.63
#define l2  8.57
#define l3  26
#define l4  26

// robot's width and length (cm) 
#define H  52.8
#define w  22.8

// initial motor position (rad)
#define theta1Init  0
#define theta2Init  0
#define theta3Init  0

// movements deltas (cm,rad)
#define forwardDelta  30;
#define crabDelta 10;
#define spinDelta pi/10;

typedef enum _Result {FAIL = 0, SUCCESS = 1} Result;

#endif // !DEFINITIONS_H