#define IX(i, j) ((i) + (N + 2) * (j))
#define SWAP(x0, x)                                                                                                    \
    {                                                                                                                  \
        float *tmp = x0;                                                                                               \
        x0 = x;                                                                                                        \
        x = tmp;                                                                                                       \
    }
#define FOR_EACH_CELL                                                                                                  \
    for (i = 1; i <= N; i++) {                                                                                         \
        for (j = 1; j <= N; j++) {
#define END_FOR                                                                                                        \
    }                                                                                                                  \
    }

#include <math.h>
#include <stdlib.h>
#include <iostream>


void add_source(int N, float *x, float *s, float dt) {
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++)
        x[i] += dt * s[i];
}

void set_bnd(int N, int b, float *x) {
    int i;

    for (i = 1; i <= N; i++) {
        x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
        x[IX(N + 1, i)] = b == 1 ? -x[IX(N, i)] : x[IX(N, i)];
        x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
        x[IX(i, N + 1)] = b == 2 ? -x[IX(i, N)] : x[IX(i, N)];
    }
    x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
    x[IX(0, N + 1)] = 0.5f * (x[IX(1, N + 1)] + x[IX(0, N)]);
    x[IX(N + 1, 0)] = 0.5f * (x[IX(N, 0)] + x[IX(N + 1, 1)]);
    x[IX(N + 1, N + 1)] = 0.5f * (x[IX(N, N + 1)] + x[IX(N + 1, N)]);
}

void lin_solve(int N, int b, float *x, float *x0, float a, float c) {
    int i, j, k;

    for (k = 0; k < 20; k++) {
        FOR_EACH_CELL
        x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
        END_FOR
        set_bnd(N, b, x);
    }
}

void diffuse(int N, int b, float *x, float *x0, float diff, float dt) {
    float a = dt * diff * N * N;
    lin_solve(N, b, x, x0, a, 1 + 4 * a);
}

void advect(int N, int b, float *d, float *d0, float *u, float *v, float dt) {
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;

    dt0 = dt * N;
    FOR_EACH_CELL
    x = i - dt0 * u[IX(i, j)];
    y = j - dt0 * v[IX(i, j)];
    if (x < 0.5f)
        x = 0.5f;
    if (x > N + 0.5f)
        x = N + 0.5f;
    i0 = (int) x;
    i1 = i0 + 1;
    if (y < 0.5f)
        y = 0.5f;
    if (y > N + 0.5f)
        y = N + 0.5f;
    j0 = (int) y;
    j1 = j0 + 1;
    s1 = x - i0;
    s0 = 1 - s1;
    t1 = y - j0;
    t0 = 1 - t1;
    d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) + s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
    END_FOR
    set_bnd(N, b, d);
}

void project(int N, float *u, float *v, float *p, float *div) {
    int i, j;

    FOR_EACH_CELL
    div[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / N;
    p[IX(i, j)] = 0;
    END_FOR
    set_bnd(N, 0, div);
    set_bnd(N, 0, p);

    lin_solve(N, 0, p, div, 1, 4);

    FOR_EACH_CELL
    u[IX(i, j)] -= 0.5f * N * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
    v[IX(i, j)] -= 0.5f * N * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
    END_FOR
    set_bnd(N, 1, u);
    set_bnd(N, 2, v);
}

// Update u and v inplace. curl is a buffer used in the function.
void add_vorticity_conf_forces(int N, float *u, float *v, float *curl, float epsilon, float dt) {
    int i, j;
    FOR_EACH_CELL
        curl[IX(i, j)] = 0.5f * (v[IX(i+1,j)] - v[IX(i-1,j)] - u[IX(i,j+1)] + u[IX(i,j-1)]) / N;
    END_FOR
    FOR_EACH_CELL
        float x = 0.5f * (abs(curl[IX(i+1, j)]) - abs(curl[IX(i-1, j)])) / N;
        float y = 0.5f * (abs(curl[IX(i, j+1)]) - abs(curl[IX(i, j-1)])) / N;
        float len = sqrt(x * x + y * y);
        if (len < 1e-6f) {
            continue;
        }
        x /= len;
        y /= len;
        float x_force = epsilon * curl[IX(i, j)] * (y);
        float y_force = epsilon * curl[IX(i, j)] * (-x);

        u[IX(i, j)] += dt * x_force;
        v[IX(i, j)] += dt * y_force;
    END_FOR
}

void dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt) {
    add_source(N, x, x0, dt);
    SWAP(x0, x);
    diffuse(N, 0, x, x0, diff, dt);
    SWAP(x0, x);
    advect(N, 0, x, x0, u, v, dt);
}

void vel_step(int N, float *u, float *v, float *u0, float *v0, float visc, float dt) {
    // vel: u,v - source forces u0, v0
    add_source(N, u, u0, dt);
    add_source(N, v, v0, dt);
    // vel: u,v - buffers u0, v0
    add_vorticity_conf_forces(N, u, v, u0, 160, dt);
    // vel: u,v - buffers u0, v0
    SWAP(u0, u);
    SWAP(v0, v);
    // vel: u0,v0 - buffers u, v
    diffuse(N, 1, u, u0, visc, dt);
    diffuse(N, 2, v, v0, visc, dt);
    // vel: u,v - buffers u0, v0
    project(N, u, v, u0, v0);
    // vel: u,v - buffers u0, v0
    SWAP(u0, u);
    SWAP(v0, v);
    // vel: u0,v0 - buffers u, v
    advect(N, 1, u, u0, u0, v0, dt);
    advect(N, 2, v, v0, u0, v0, dt);
    // vel: u,v - buffers u0, v0
    project(N, u, v, u0, v0);
    // vel: u,v - buffers u0, v0
}
