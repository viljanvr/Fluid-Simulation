#include <math.h>
#include <stdlib.h>
#include <vector>
#include "RectangleObstacle.h"
#include "gfx/vec2.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

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
#define FOR_EACH_CELL_WITH_BOUNDARY                                                                                    \
    for (i = 0; i <= N + 1; i++) {                                                                                     \
        for (j = 0; j <= N + 1; j++) {
#define END_FOR                                                                                                        \
    }                                                                                                                  \
    }


void add_source(int N, float *x, float *s, float dt) {
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++) {
        x[i] += dt * s[i];
    }
}

void add_temp_forces(int N, float *v, float *temp, float dt) {
    int i, size = (N + 2) * (N + 2);
    for (i = 0; i < size; i++) {
        v[i] += dt * temp[i] * 0.0001;
    }
}

void set_bnd(int N, int b, float *x, RectangleObstacle **obstacle_mask) {
    int i, j;

    // // // Update the outer cells (i.e. border)
    // for (i = 1; i <= N; i++) {
    //     x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
    //     x[IX(N + 1, i)] = b == 1 ? -x[IX(N, i)] : x[IX(N, i)];
    //     x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
    //     x[IX(i, N + 1)] = b == 2 ? -x[IX(i, N)] : x[IX(i, N)];
    // }
    // x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
    // x[IX(0, N + 1)] = 0.5f * (x[IX(1, N + 1)] + x[IX(0, N)]);
    // x[IX(N + 1, 0)] = 0.5f * (x[IX(N, 0)] + x[IX(N + 1, 1)]);
    // x[IX(N + 1, N + 1)] = 0.5f * (x[IX(N, N + 1)] + x[IX(N + 1, N)]);


    FOR_EACH_CELL_WITH_BOUNDARY;
    if (i > 0 && i < N + 1 && j > 0 && j < N + 1 && !obstacle_mask[IX(i, j)]) {
        continue;
    }

    // special cases for temperature
    if (b == 3 && (j == N + 1 || i == 0 || i == N + 1)) {
        x[IX(i, j)] = 0.0;
        continue;
    }
    if (b == 3 && j == 0 && (i % 16) == 8) {
        x[IX(i, j)] = 500.0f;
        continue;
    }

    int fluid_neighbor_count = 0;
    float sum = 0.0;
    Vec2f velocityAtPosition = 0;
    if (obstacle_mask[IX(i, j)]) {
        velocityAtPosition =
                obstacle_mask[IX(i, j)]->getVelocityFromPosition(((float) i + 0.5) / N, ((float) j + 0.5) / N);
    }
    if (i > 0 && !obstacle_mask[IX(i - 1, j)]) {
        sum += b == 1 ? -x[IX(i - 1, j)] + 2 * velocityAtPosition[0] : x[IX(i - 1, j)];
        fluid_neighbor_count += 1;
    }
    if (i < N + 1 && !obstacle_mask[IX(i + 1, j)]) {
        sum += b == 1 ? -x[IX(i + 1, j)] + 2 * velocityAtPosition[0] : x[IX(i + 1, j)];
        fluid_neighbor_count += 1;
    }
    if (j < N + 1 && !obstacle_mask[IX(i, j + 1)]) {
        sum += b == 2 ? -x[IX(i, j + 1)] + 2 * velocityAtPosition[1] : x[IX(i, j + 1)];
        fluid_neighbor_count += 1;
    }
    if (j > 0 && !obstacle_mask[IX(i, j - 1)]) {
        sum += b == 2 ? -x[IX(i, j - 1)] + 2 * velocityAtPosition[1] : x[IX(i, j - 1)];
        fluid_neighbor_count += 1;
    }
    if (fluid_neighbor_count > 0) {
        x[IX(i, j)] = sum / fluid_neighbor_count;
    } else {
        x[IX(i, j)] = 0.0;
    }
    END_FOR
}

void lin_solve(int N, int b, float *x, float *x0, float a, float c, RectangleObstacle **obstacle_mask) {
    int i, j, k;

    for (k = 0; k < 50; k++) {
        FOR_EACH_CELL
        x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) / c;
        END_FOR
        set_bnd(N, b, x, obstacle_mask);
    }
}

void diffuse(int N, int b, float *x, float *x0, float diff, float dt, RectangleObstacle **obstacle_mask) {
    float a = dt * diff * N * N;
    lin_solve(N, b, x, x0, a, 1 + 4 * a, obstacle_mask);
}

void advect(int N, int b, float *d, float *d0, float *u, float *v, float dt, RectangleObstacle **obstacle_mask,
            const std::vector<RectangleObstacle *> &obstacle_list) {
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;
    dt0 = dt * N;
    FOR_EACH_CELL

    x = i - dt0 * u[IX(i, j)];
    y = j - dt0 * v[IX(i, j)];
    for (const auto &obj: obstacle_list) {
        auto intersection = obj->get_line_intersection(Vec2f((i - 0.5f) / N, (j - 0.5f) / N),
                                                       Vec2f((x - 0.5f) / N, (y - 0.5f) / N));
        if (intersection.has_value()) {
            x = (intersection.value()[0] * N) + 0.5f;
            y = (intersection.value()[1] * N) + 0.5f;
        }
    }

    if (x < 0.5f)
        x = 0.5f;
    if (x > N + 0.5f)
        x = N + 0.5f;
    if (y < 0.5f)
        y = 0.5f;
    if (y > N + 0.5f)
        y = N + 0.5f;
    i0 = (int) x;
    i1 = i0 + 1;
    j0 = (int) y;
    j1 = j0 + 1;
    s1 = x - i0;
    s0 = 1 - s1;
    t1 = y - j0;
    t0 = 1 - t1;

    d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) + s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);

    END_FOR
    set_bnd(N, b, d, obstacle_mask);
}

void applyPressureForces(int N, float *p, RectangleObstacle **obstacle_mask) {
    int i, j;
    float h = 1.0f / N;
    float coupling_strength = 3.0f;
    FOR_EACH_CELL {
        RectangleObstacle *ob = obstacle_mask[IX(i, j)];
        if (!ob)
            continue;

        Vec2f pos = Vec2f((i + 0.5f) / N, (j + 0.5f) / N);

        if (i > 2 && !obstacle_mask[IX(i - 1, j)]) {
            float faceP = 0.5f * (p[IX(i, j)] + p[IX(i - 1, j)]);
            Vec2f dF = -faceP * Vec2f(-1, 0) * h * coupling_strength;
            ob->addForceAtPosition(dF, pos);
        }
        if (i < N - 1 && !obstacle_mask[IX(i + 1, j)]) {
            float faceP = 0.5f * (p[IX(i, j)] + p[IX(i + 1, j)]);
            Vec2f dF = -faceP * Vec2f(1, 0) * h * coupling_strength;
            ob->addForceAtPosition(dF, pos);
        }
        if (j > 2 && !obstacle_mask[IX(i, j - 1)]) {
            float faceP = 0.5f * (p[IX(i, j)] + p[IX(i, j - 1)]);
            Vec2f dF = -faceP * Vec2f(0, -1) * h * coupling_strength;
            ob->addForceAtPosition(dF, pos);
        }
        if (j < N - 1 && !obstacle_mask[IX(i, j + 1)]) {
            float faceP = 0.5f * (p[IX(i, j)] + p[IX(i, j + 1)]);
            Vec2f dF = -faceP * Vec2f(0, 1) * h * coupling_strength;
            ob->addForceAtPosition(dF, pos);
        }
    }
    END_FOR;
}


void project(int N, float *u, float *v, float *p, float *div, RectangleObstacle **obstacle_mask) {
    int i, j;

    FOR_EACH_CELL
    div[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]) / N;
    p[IX(i, j)] = 0;
    END_FOR
    set_bnd(N, 0, div, obstacle_mask);
    set_bnd(N, 0, p, obstacle_mask);

    lin_solve(N, 0, p, div, 1, 4, obstacle_mask);


    FOR_EACH_CELL
    u[IX(i, j)] -= 0.5f * N * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
    v[IX(i, j)] -= 0.5f * N * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
    END_FOR
    set_bnd(N, 1, u, obstacle_mask);
    set_bnd(N, 2, v, obstacle_mask);
}

// Update u and v inplace. curl is a buffer used in the function.
void add_vorticity_conf_forces(int N, float *u, float *v, float *curl, float epsilon, float dt) {
    if (!epsilon)
        return;
    int i, j;

    FOR_EACH_CELL
    curl[IX(i, j)] = 0.5f * (v[IX(i + 1, j)] - v[IX(i - 1, j)] - u[IX(i, j + 1)] + u[IX(i, j - 1)]) / N;
    END_FOR

    FOR_EACH_CELL
    float x = 0.5f * (abs(curl[IX(i + 1, j)]) - abs(curl[IX(i - 1, j)])) / N;
    float y = 0.5f * (abs(curl[IX(i, j + 1)]) - abs(curl[IX(i, j - 1)])) / N;
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

void dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt, RectangleObstacle **obstacle_mask,
               const std::vector<RectangleObstacle *> &obstacle_list) {
    add_source(N, x, x0, dt);
    SWAP(x0, x);
    diffuse(N, 0, x, x0, diff, dt, obstacle_mask);
    SWAP(x0, x);
    advect(N, 0, x, x0, u, v, dt, obstacle_mask, obstacle_list);
}

void temp_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt, RectangleObstacle **obstacle_mask,
               const std::vector<RectangleObstacle *> &obstacle_list) {
    add_source(N, x, x0, dt);
    SWAP(x0, x);
    diffuse(N, 3, x, x0, diff, dt, obstacle_mask);
    SWAP(x0, x);
    advect(N, 3, x, x0, u, v, dt, obstacle_mask, obstacle_list);
}


void vel_step(int N, float *u, float *v, float *u0, float *v0, float *temp, float visc, float dt,
              float vorticity_conf_epsilon, bool pressure_force_enabled, RectangleObstacle **obstacle_mask,
              const std::vector<RectangleObstacle *> &obstacle_list) {
    // vel: u,v - source forces u0, v0
    add_source(N, u, u0, dt);
    add_source(N, v, v0, dt);
    // vel: u,v - buffers u0, v0
    add_temp_forces(N, v, temp, dt);
    add_vorticity_conf_forces(N, u, v, u0, vorticity_conf_epsilon, dt);
    // vel: u,v - buffers u0, v0
    project(N, u, v, u0, v0, obstacle_mask);
    if (pressure_force_enabled)
        applyPressureForces(N, u0, obstacle_mask);
    // vel: u,v - buffers u0(stores pressure), v0
    // vel: u,v - buffers u0, v0
    SWAP(u0, u);
    SWAP(v0, v);
    // vel: u0,v0 - buffers u, v
    diffuse(N, 1, u, u0, visc, dt, obstacle_mask);
    diffuse(N, 2, v, v0, visc, dt, obstacle_mask);
    // vel: u,v - buffers u0, v0
    project(N, u, v, u0, v0, obstacle_mask);
    if (pressure_force_enabled)
        applyPressureForces(N, u0, obstacle_mask);
    // vel: u,v - buffers u0(stores pressure), v0
    SWAP(u0, u);
    SWAP(v0, v);
    // vel: u0,v0 - buffers u, v
    advect(N, 1, u, u0, u0, v0, dt, obstacle_mask, obstacle_list);
    advect(N, 2, v, v0, u0, v0, dt, obstacle_mask, obstacle_list);
    // vel: u,v - buffers u0, v0
    project(N, u, v, u0, v0, obstacle_mask);
    if (pressure_force_enabled)
        applyPressureForces(N, u0, obstacle_mask);
    // vel: u,v - buffers u0(stores pressure), v0
}
