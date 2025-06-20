#pragma once
#include "linearSolver.h"

class ConjGradMatrix : public implicitMatrix {
public:
    ConjGradMatrix(int a, int c, int N);
    void matVecMult(float x[], float r[]) override;
private:
    int m_a;
    int m_c;
    int m_N;
};