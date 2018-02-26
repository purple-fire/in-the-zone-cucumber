#include <main.h>


#define ABS(x) ( (x)>=0?(x):-(x) )

//Motor Power Bounds
#define maxOutput	127
#define minOutput 10

//Storing Constants for Wheel Diameter and Pi
#define wheelDiameter 4
#define Pi 3.14159

// Stick L/R, Axis X/Y
#define CRY						                2
#define CLY						                3
#define CRX							              1
#define CLX								            4

int limitMotorPower(int power);
