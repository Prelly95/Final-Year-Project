#include "undistortPoint.h"
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Input variables
	double *xIn;
	double *yIn;
    double *cameraMatrix;
	double *distortionParams;

    // Output variables
	double *xOut;
	double *yOut;

	// Size variables
    varint nx, ny;

    // Check that we have the correct number of arguments
    if (nrhs != 4) // we need x, y, camera matrix, distortion params
    {
        mexErrMsgTxt("Expects 4 input arguments: x, y, cameraMatrix, distortionParama");
        return;
    }
    if (nlhs > 2) // we need to output x, lm and error
    {
        mexErrMsgTxt("Too many output arguments.");
        return;
    }

	nx = mxGetM(prhs[0]); // size(H)
	ny = mxGetM(prhs[1]); // size(H)

	if(nx != ny)
	{
		mexErrMsgTxt("X and Y must be the same size");
		return;
	}

	xIn = mxGetPr(prhs[0]);
	yIn = mxGetPr(prhs[1]);
	cameraMatrix = mxGetPr(prhs[2]);
	distortionParams = mxGetPr(prhs[3]);

	plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
	plhs[1] = mxCreateDoubleMatrix(ny, 1, mxREAL);
	xOut = mxGetPr(plhs[0]);
	yOut = mxGetPr(plhs[1]);

    undistortCorners(xOut, yOut, xIn, yIn, nx, cameraMatrix, distortionParams);

	// mexPrintf("xOut = %f\n", xOut);
	// mexPrintf("yOut = %f\n", yOut);
	return;


}

/*-------------------------------------------------------------------------*/
void printvector(varint size, const double *vec, const char *name)
{
    varint i;
    if (size > 0)
    {
        mexPrintf("\n ");
        mexPrintf(name);
        mexPrintf(" = \n\n");
        for (i = 0; i < size; i++)
        {
            if (vec[i] >= 0.0)
            {
                mexPrintf("          %3.5e\n", vec[i]);
            }
            else
            {
                mexPrintf("         %3.5e\n", vec[i]);
            }
        }
        mexPrintf("\n\n");
    }
    else
    {
        mexPrintf("\n ");
        mexPrintf(name);
        mexPrintf("= []");
    }
}
/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
void printmatrix(varint m, varint n, const double *A, varint lda,
                 const char *name)
{
    varint i, j;
    if (m * n > 0)
    {
        mexPrintf("\n ");
        mexPrintf(name);
        mexPrintf(" = \n\n");
        for (i = 0; i < m; i++)
        {
            for (j = 0; j < n; j++)
            {
                if (A[i + j * lda] > 0.0)
                {
                    mexPrintf("   %3.4e", A[i + j * lda]);
                }
                // if (A[i + j * lda] > 0.0)
                // {
                //     mexPrintf("   %.2f", A[i + j * lda]);
                // }
                else if (A[i + j * lda] < 0.0)
                {
                    mexPrintf("  %3.4e", A[i + j * lda]);
                }
                // else if (A[i + j * lda] < 0.0)
                // {
                //     mexPrintf("  %.2f", A[i + j * lda]);
                // }
                else
                {
                    mexPrintf("   0         ");
                }
            }
            mexPrintf("\n");
        }
        mexPrintf("\n\n");
    }
    else
    {
        mexPrintf("\n ");
        mexPrintf(name);
        mexPrintf("= []");
    }
}
/*-------------------------------------------------------------------------*/