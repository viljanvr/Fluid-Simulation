#ifndef LINEAR_SOLVER_H
#define LINEAR_SOLVER_H

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

// Karen's CGD

#define MAX_STEPS 100

// Matrix class the solver will accept
class implicitMatrix {
public:
    virtual void matVecMult(float x[], float r[]) = 0;
};

// Matrix class the solver will accept
class implicitMatrixWithTrans : public implicitMatrix {
public:
    virtual void matVecMult(float x[], float r[]) = 0;
    virtual void matTransVecMult(float x[], float r[]) = 0;
};


// Solve Ax = b for a symmetric, positive definite matrix A
// A is represented implicitely by the function "matVecMult"
// which performs a matrix vector multiple Av and places result in r
// "n" is the length of the vectors x and b
// "epsilon" is the error tolerance
// "steps", as passed, is the maximum number of steps, or 0 (implying MAX_STEPS)
// Upon completion, "steps" contains the number of iterations taken
float ConjGrad(int n, implicitMatrix *A, float x[], float b[],
                float epsilon, // how low should we go?
                int *steps);

// Some vector helper functions
void vecAddEqual(int n, float r[], float v[]);
void vecDiffEqual(int n, float r[], float v[]);
void vecAssign(int n, float v1[], float v2[]);
void vecTimesScalar(int n, float v[], float s);
float vecDot(int n, float v1[], float v2[]);
float vecSqrLen(int n, float v[]);

#endif
