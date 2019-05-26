#ifndef UNDIST
#define UNDIST

#include "mex.h"

#define varint mwSignedIndex
#define idx2D(M, I, J) ((J * M) + I)
#define idx3D(M, N, I, J, T) ((T * M * N) + (J * M) + I)

#define ITTER 10

#define CX 2
#define CY 5
#define FX 0
#define FY 4

#define K1 0
#define K2 1
#define K3 4
#define K4 5
#define K5 6
#define K6 7

#define P1 2
#define P2 3

//  Main src functions
void undistortCorners(double *xOut, double *yOut, double *xIn, double *yIn, varint n, double *cameraIntrinsics, double *cameraDistortion);
void calculateUndistortion(double *xOut, double *yOut, double x, double y, double *k, double *p);

// Mex Functions
void printmatrix(varint m, varint n, const double *A, varint lda, const char *name);
void printvector(varint size, const double *vec, const char *name);


#endif