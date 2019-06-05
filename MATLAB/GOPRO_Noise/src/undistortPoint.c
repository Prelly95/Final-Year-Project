#include <math.h>
#include "undistortPoint.h"

void undistortCorners(double *xOut, double *yOut, double *xIn, double *yIn, varint n, double *cameraIntrinsics, double *cameraDistortion)
{
	// * Inputs to the function working
	
	int ii = 0;
	double k[14]={0};
	double p[2]={0};

	double fx = cameraIntrinsics[FX];
	double fy = cameraIntrinsics[FY];

	double cx = cameraIntrinsics[CX];
	double cy = cameraIntrinsics[CY];

	k[0] = cameraDistortion[K1];
	k[1] = cameraDistortion[K2];
	k[2] = cameraDistortion[K3];
	k[3] = cameraDistortion[K4];
	k[4] = cameraDistortion[K5];
	k[5] = cameraDistortion[K6];

	p[0] = cameraDistortion[P1];
	p[1] = cameraDistortion[P2];
	
	for(ii = 0; ii < n; ii++)
	{
		double x = (xIn[ii] - cx) / fx;
		double y = (yIn[ii] - cy) / fy;
		calculateUndistortion(&xOut[ii], &yOut[ii], x, y, k, p);
		xOut[ii] = (xOut[ii]*fx + cx);
		yOut[ii] = (yOut[ii]*fy + cy);
	}
	
	return;
}

void calculateUndistortion(double *xOut, double *yOut, double x, double y, double *k, double *p)
{
	int ii = 0;

	double x0 = x;
	double y0 = y;
	
	double r2, r4, r6;

	double iRaidDist; // Inverse radial distortion
	double tangDistX; // Tangental distortion in the x direction
	double tangDistY; // Tangental distortion in the y direction

	for(ii = 0; ii < ITTER; ii++)
	{
		r2 = x * x + y * y;
		r4 = r2 * r2;
		r6 = r4 * r2;
		iRaidDist = (1 + (k[5] * r6 + k[4] * r4 + k[3] * r2)) / (1 + (k[2] * r6 + k[1] * r4 + k[0] * r2));
		tangDistX = 2 * p[0] * x * y + p[1] * (r2 + 2 * x * x);
		tangDistY = p[0] * (r2 + 2 * y * y) + 2 * p[1] * x * y;

		x = (x0 - tangDistX) * iRaidDist;
		y = (y0 - tangDistY) * iRaidDist;
	}

	*xOut = x;
	*yOut = y;
}