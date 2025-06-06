/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

        This code is a simple prototype that demonstrates how to use the
        code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
        for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include <cctype>
#include <cmath>
#include <iostream>
#include <ostream>
#include <stdio.h>
#include <stdlib.h>
#include "jet_colormap.h"
#include "viridis_colormap.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

/* macros */

#define IX(i, j) ((i) + (N + 2) * (j))

/* external definitions (from solver.c) */

extern void dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt, bool *obstacles);
extern void vel_step(int N, float *u, float *v, float *u0, float *v0, float visc, float dt, bool *obstacles);

/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;

static float *u, *v, *u_prev, *v_prev;
static float *dens, *dens_prev;
static bool *obstacles;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;

static float (*colormap)[3] = jet_colormap;

/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/

static void free_data(void) {
    if (u)
        free(u);
    if (v)
        free(v);
    if (u_prev)
        free(u_prev);
    if (v_prev)
        free(v_prev);
    if (dens)
        free(dens);
    if (dens_prev)
        free(dens_prev);
    if (obstacles)
        free(obstacles);
}

static void clear_data(void) {
    int i, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
        obstacles[i] = false;
    }
    for (size_t i = 0; i <= N; i++) {
        obstacles[IX(i, 0)] = true;
        obstacles[IX(N+1, i)] = true;
        obstacles[IX(N+1 - i, N+1)] = true;
        obstacles[IX(0, N+1 - i)] = true;
    }
    for (size_t i = N / 2 - 10; i < N / 2; i++) {
        for (size_t j = 1; j < N / 2 - 5; j++) {
            obstacles[IX(i, j)] = true;
        }
        for (size_t j = N - 10; j > N / 2 + 5; j--) {
            obstacles[IX(i, j)] = true;
        }
    }

    //for (size_t i = 20; i < 30; i++) {
    //    for (size_t j = 10; j < 20; j++) {
    //        obstacles[IX(i, j)] = true;
    //    }
    //}

}

static int allocate_data(void) {
    int size = (N + 2) * (N + 2);

    u = (float *) malloc(size * sizeof(float));
    v = (float *) malloc(size * sizeof(float));
    u_prev = (float *) malloc(size * sizeof(float));
    v_prev = (float *) malloc(size * sizeof(float));
    dens = (float *) malloc(size * sizeof(float));
    dens_prev = (float *) malloc(size * sizeof(float));
    obstacles = (bool *) malloc(size * sizeof(bool));

    if (!u || !v || !u_prev || !v_prev || !dens || !dens_prev || !obstacles) {
        fprintf(stderr, "cannot allocate data\n");
        return (0);
    }

    return (1);
}

/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display(void) {
    glViewport(0, 0, win_x, win_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void) { glutSwapBuffers(); }

static void from_colormap(float t, float max, float *r, float *g, float *b) {
    int idx = (int) (t * 255.0f / max);
    idx = idx < 0 ? 0 : (idx > 255 ? 255 : idx);
    *r = colormap[idx][0];
    *g = colormap[idx][1];
    *b = colormap[idx][2];
}

static void color_on_direction(float u, float v, float &r, float &g, float &b) {
    r = std::max(u, 0.0f) + std::max(v, 0.0f);
    g = std::max(u, 0.0f) + std::max(-u, 0.0f) * 0.3f + std::max(-v, 0.0f) * 0.5;
    b = std::max(-u, 0.0f);
    r = sqrt(r);
    g = sqrt(g);
    b = sqrt(b);
    //r = std::max(v, 0.0f) + std::max(u, 0.0f);
    //g = std::abs(u);
    //b = std::max(-v, 0.0f);
    float magnitude = std::sqrt(r * r + g * g + b * b);
    float target_magnitude = std::clamp(std::sqrt(u * u + v * v) * 20, 0.45f, 1.3f);
    r = r / magnitude * target_magnitude;
    g = g / magnitude * target_magnitude;
    b = b / magnitude * target_magnitude;
}

static void draw_velocity(void) {
    int i, j;
    float x, y, h;
    float r, g, b;

    h = 1.0f / N;

    glLineWidth(1.0f);

    glBegin(GL_LINES);

    for (i = 1; i <= N; i++) {
        x = (i - 0.5f) * h;
        for (j = 1; j <= N; j++) {
            y = (j - 0.5f) * h;
            //float magnitude = std::sqrt(u[IX(i, j)] * u[IX(i, j)] + v[IX(i, j)] * v[IX(i, j)]);
            //from_colormap(magnitude, 0.15f, &r, &g, &b);
            color_on_direction(u[IX(i,j)], v[IX(i,j)], r, g, b);
            glColor3f(r, g, b);
            glVertex2f(x, y);
            glVertex2f(x + u[IX(i, j)], y + v[IX(i, j)]);
        }
    }

    glEnd();
}

static void draw_density(void) {
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 1.0f / N;

    glBegin(GL_QUADS);

    for (i = 0; i <= N; i++) {
        x = (i - 0.5f) * h;
        for (j = 0; j <= N; j++) {
            y = (j - 0.5f) * h;

            //d00 = obstacles[IX(i,j)] ? 1 : dens[IX(i, j)];
            //d01 = obstacles[IX(i,j + 1)] ? 1 : dens[IX(i, j + 1)];
            //d10 = obstacles[IX(i + 1,j)] ? 1 : dens[IX(i + 1, j)];
            //d11 = obstacles[IX(i + 1,j + 1)] ? 1 : dens[IX(i + 1, j + 1)];

            d00 = dens[IX(i, j)];
            d01 = dens[IX(i, j + 1)];
            d10 = dens[IX(i + 1, j)];
            d11 = dens[IX(i + 1, j + 1)];

            glColor3f(d00, d00, d00);
            glVertex2f(x, y);
            glColor3f(d10, d10, d10);
            glVertex2f(x + h, y);
            glColor3f(d11, d11, d11);
            glVertex2f(x + h, y + h);
            glColor3f(d01, d01, d01);
            glVertex2f(x, y + h);
        }
    }

    glEnd();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI(float *d, float *u, float *v) {
    int i, j, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++) {
        u[i] = v[i] = d[i] = 0.0f;
    }

    if (!mouse_down[0] && !mouse_down[2])
        return;

    i = (int) ((mx / (float) win_x) * N + 1);
    j = (int) (((win_y - my) / (float) win_y) * N + 1);

    if (i < 1 || i > N || j < 1 || j > N)
        return;

    if (mouse_down[0]) {
        u[IX(i, j)] = force * (mx - omx);
        v[IX(i, j)] = force * (omy - my);
    }

    if (mouse_down[2]) {
        d[IX(i, j)] = source;
    }

    omx = mx;
    omy = my;

    return;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y) {
    switch (key) {
        case 'c':
        case 'C':
            clear_data();
            break;

        case 'q':
        case 'Q':
            free_data();
            exit(0);
            break;

        case 'v':
        case 'V':
            dvel = !dvel;
            break;
        default:
            if (std::isdigit(key)) {
                // If we want to use scenes like in project 1
                // int n = key - '0';
                // set_scene(n);

                dens[IX(N / 2, N / 2)] = 100.0f;
                u[IX(N / 2, N / 2)] = 5.0f;
                v[IX(N / 2, N / 2)] = 0.0f;
            }
    }
}

static void mouse_func(int button, int state, int x, int y) {
    omx = mx = x;
    omx = my = y;

    mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func(int x, int y) {
    mx = x;
    my = y;
}

static void reshape_func(int width, int height) {
    glutSetWindow(win_id);
    glutReshapeWindow(width, height);

    win_x = width;
    win_y = height;
}

static void idle_func(void) {
    get_from_UI(dens_prev, u_prev, v_prev);
    vel_step(N, u, v, u_prev, v_prev, visc, dt, obstacles);
    dens_step(N, dens, dens_prev, u, v, diff, dt, obstacles);

    glutSetWindow(win_id);
    glutPostRedisplay();
}

static void display_func(void) {
    pre_display();

    if (dvel)
        draw_velocity();
    else
        draw_density();

    post_display();
}

/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window(void) {
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

    glutInitWindowPosition(0, 0);
    glutInitWindowSize(win_x, win_y);
    win_id = glutCreateWindow("Alias | wavefront");

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();

    pre_display();

    glutKeyboardFunc(key_func);
    glutMouseFunc(mouse_func);
    glutMotionFunc(motion_func);
    glutReshapeFunc(reshape_func);
    glutIdleFunc(idle_func);
    glutDisplayFunc(display_func);
}

/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main(int argc, char **argv) {
    glutInit(&argc, argv);

    if (argc != 1 && argc != 7) {
        fprintf(stderr, "usage : %s N dt diff visc force source\n", argv[0]);
        fprintf(stderr, "where:\n");
        fprintf(stderr, "\t N      : grid resolution\n");
        fprintf(stderr, "\t dt     : time step\n");
        fprintf(stderr, "\t diff   : diffusion rate of the density\n");
        fprintf(stderr, "\t visc   : viscosity of the fluid\n");
        fprintf(stderr, "\t force  : scales the mouse movement that generate a force\n");
        fprintf(stderr, "\t source : amount of density that will be deposited\n");
        exit(1);
    }

    if (argc == 1) {
        N = 64;
        dt = 0.1f;
        diff = 0.0f;
        visc = 0.0f;
        force = 5.0f;
        source = 100.0f;
        fprintf(stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n", N, dt, diff, visc, force,
                source);
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        diff = atof(argv[3]);
        visc = atof(argv[4]);
        force = atof(argv[5]);
        source = atof(argv[6]);
    }

    printf("\n\nHow to use this demo:\n\n");
    printf("\t Add densities with the right mouse button\n");
    printf("\t Add velocities with the left mouse button and dragging the mouse\n");
    printf("\t Toggle density/velocity display with the 'v' key\n");
    printf("\t Clear the simulation by pressing the 'c' key\n");
    printf("\t Quit by pressing the 'q' key\n");

    dvel = 0;

    if (!allocate_data())
        exit(1);
    clear_data();

    win_x = 512;
    win_y = 512;
    open_glut_window();

    glutMainLoop();

    exit(0);
}
