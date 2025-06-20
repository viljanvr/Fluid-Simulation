// ----------------------------
// Code from assignment 1
// -----------------------------

#include "linearSolver.h"

// vector helper functions

void vecAddEqual(int n, float r[], float v[]) {
    for (int i = 0; i < n; i++)
        r[i] = r[i] + v[i];
}

void vecDiffEqual(int n, float r[], float v[]) {
    for (int i = 0; i < n; i++)
        r[i] = r[i] - v[i];
}

void vecAssign(int n, float v1[], float v2[]) {
    for (int i = 0; i < n; i++)
        v1[i] = v2[i];
}

void vecTimesScalar(int n, float v[], float s) {
    for (int i = 0; i < n; i++)
        v[i] *= s;
}

float vecDot(int n, float v1[], float v2[]) {
    float dot = 0;
    for (int i = 0; i < n; i++)
        dot += v1[i] * v2[i];
    return dot;
}

float vecSqrLen(int n, float v[]) { return vecDot(n, v, v); }

float ConjGrad(int n, implicitMatrix *A, float x[], float b[],
               float epsilon, // how low should we go?
               int *steps) {
    int i, iMax;
    float alpha, beta, rSqrLen, rSqrLenOld, u;

    float *r = (float *) malloc(sizeof(float) * n);
    float *d = (float *) malloc(sizeof(float) * n);
    float *t = (float *) malloc(sizeof(float) * n);
    float *temp = (float *) malloc(sizeof(float) * n);

    vecAssign(n, x, b);

    vecAssign(n, r, b);
    A->matVecMult(x, temp);

    vecDiffEqual(n, r, temp);

    rSqrLen = vecSqrLen(n, r);

    vecAssign(n, d, r);

    i = 0;
    if (*steps)
        iMax = *steps;
    else
        iMax = MAX_STEPS;

    if (rSqrLen > epsilon)
        while (i < iMax) {
            i++;
            A->matVecMult(d, t);
            u = vecDot(n, d, t);

            if (u == 0) {
                printf("(SolveConjGrad) d'Ad = 0\n");
                break;
            }

            // How far should we go?
            alpha = rSqrLen / u;

            // Take a step along direction d
            vecAssign(n, temp, d);
            vecTimesScalar(n, temp, alpha);
            vecAddEqual(n, x, temp);

            if (i & 0x3F) {
                vecAssign(n, temp, t);
                vecTimesScalar(n, temp, alpha);
                vecDiffEqual(n, r, temp);
            } else {
                // For stability, correct r every 64th iteration
                vecAssign(n, r, b);
                A->matVecMult(x, temp);
                vecDiffEqual(n, r, temp);
            }

            rSqrLenOld = rSqrLen;
            rSqrLen = vecSqrLen(n, r);

            // Converged! Let's get out of here
            if (rSqrLen <= epsilon)
                break;

            // Change direction: d = r + beta * d
            beta = rSqrLen / rSqrLenOld;
            vecTimesScalar(n, d, beta);
            vecAddEqual(n, d, r);
        }

    // free memory

    free(r);
    free(d);
    free(t);
    free(temp);

    *steps = i;
    return (rSqrLen);
}
