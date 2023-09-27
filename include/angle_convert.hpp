#ifndef __ANGLE_CONVERT
#define __ANGLE_CONVERT

#include <math.h>

// double *getThetaBeta(double phi_R, double phi_L);
void getThetaBeta(double *tb, double phi_R, double phi_L);

// double *getPhiVector(double theta, double beta);
void getPhiVector(double *phi, double theta, double beta);

double deg2rad(double deg);

#endif