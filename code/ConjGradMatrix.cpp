#include "ConjGradMatrix.h"

#define IX(i, j) ((i) + (m_N + 2) * (j))
#define FOR_EACH_CELL                                                                                                  \
    for (i = 1; i <= m_N; i++) {                                                                                         \
        for (j = 1; j <= m_N; j++) {
#define END_FOR                                                                                                        \
    }                                                                                                                  \
    }

ConjGradMatrix::ConjGradMatrix(int a, int c, int N) : m_a(a), m_c(c), m_N(N) {}

void ConjGradMatrix::matVecMult(float x[], float r[]) {
    int i, j;
    FOR_EACH_CELL
        r[IX(i, j)] = m_c * x[IX(i, j)] - m_a * (
            x[IX(i + 1, j)] + x[IX(i - 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)]);
    END_FOR
};