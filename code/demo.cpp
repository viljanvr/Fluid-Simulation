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

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "RectangleObstacle.h"
#include "gfx/vec2.h"
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

extern void dens_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt,
                      RectangleObstacle **obstacle_mask, const std::vector<RectangleObstacle *> &obstacle_list);
extern void vel_step(int N, float *u, float *v, float *u0, float *v0, float visc, float dt,
                     float vorticity_conf_epsilon, RectangleObstacle **obstacle_mask,
                     const std::vector<RectangleObstacle *> &obstacle_list);

/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;
static float vorticity_conf_epsilon = 160;
static bool vorticity_conf_enabled = true;

static float *u, *v, *u_prev, *v_prev;
static float *dens, *dens_prev;

static int win_id;
static int win_x, win_y; // Window with in pixels
static int mouse_down[3];
static int omx, omy, mx, my; // Pixel-coordinate

static float (*colormap)[3] = jet_colormap;

static RectangleObstacle **obstacle_mask;
static std::vector<RectangleObstacle *> obstacles;
static RectangleObstacle *interacting_obstacle;
static Vec2f interaction_object_pos;

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
    if (obstacle_mask)
        free(obstacle_mask);
    for (auto o: obstacles) {
        free(o);
    }
}

static void clear_data(void) {
    int i, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
        obstacle_mask[i] = nullptr;
    }

    for (auto o: obstacles) {
        delete o;
    }
    obstacles.clear();

    // obstacles.push_back(new SolidCircle(N, Vec2f(0.5, 0.5), 0.07));
    //  obstacles.push_back(new RectangleObstacle(N, 2, 2, 8, 8));
    // obstacles.push_back(new RectangleObstacle(N, Vec2f(0.40f, 0.40f), Vec2f(0.60f, 0.60)));
    obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.5f), 0.5f, 0.05f, 1.0f));
    obstacles.push_back(new RectangleObstacle(N, Vec2f(0.5f, 0.3f), 0.1f, 0.10f, 1.0f));
    obstacles.push_back(new RectangleObstacle(N, Vec2f(0.2f, 0.3f), 0.1f, 0.10f, 1.0f));
    obstacles.push_back(new RectangleObstacle(N, Vec2f(0.8f, 0.3f), 0.1f, 0.10f, 1.0f));
    // obstacles.push_back(new SolidBoundary(N));
    for (i = 0; i < obstacles.size(); i++) {
        obstacles[i]->addToObstacleMask(N, obstacle_mask);
    }
}

static int allocate_data(void) {
    int size = (N + 2) * (N + 2);

    u = (float *) malloc(size * sizeof(float));
    v = (float *) malloc(size * sizeof(float));
    u_prev = (float *) malloc(size * sizeof(float));
    v_prev = (float *) malloc(size * sizeof(float));
    dens = (float *) malloc(size * sizeof(float));
    dens_prev = (float *) malloc(size * sizeof(float));
    obstacle_mask = (RectangleObstacle **) malloc(size * sizeof(RectangleObstacle *));

    if (!u || !v || !u_prev || !v_prev || !dens || !dens_prev || !obstacle_mask) {
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
    // r = std::max(v, 0.0f) + std::max(u, 0.0f);
    // g = std::abs(u);
    // b = std::max(-v, 0.0f);
    float magnitude = std::sqrt(r * r + g * g + b * b);
    float target_magnitude = std::clamp(std::sqrt(u * u + v * v) * 20, 0.45f, 1.3f);
    r = r / magnitude * target_magnitude;
    g = g / magnitude * target_magnitude;
    b = b / magnitude * target_magnitude;
}

static void draw_obstacles() {
    for (auto o: obstacles) {
        o->draw();
    }
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
            // float magnitude = std::sqrt(u[IX(i, j)] * u[IX(i, j)] + v[IX(i, j)] * v[IX(i, j)]);
            // from_colormap(magnitude, 0.15f, &r, &g, &b);
            color_on_direction(u[IX(i, j)], v[IX(i, j)], r, g, b);
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

            // d00 = obstacles[IX(i,j)] ? 1 : dens[IX(i, j)];
            // d01 = obstacles[IX(i,j + 1)] ? 1 : dens[IX(i, j + 1)];
            // d10 = obstacles[IX(i + 1,j)] ? 1 : dens[IX(i + 1, j)];
            // d11 = obstacles[IX(i + 1,j + 1)] ? 1 : dens[IX(i + 1, j + 1)];

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

static void draw_interaction(void) {
    if (interacting_obstacle) {
        glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 0.0f);
        Vec2f object_world_pos = interacting_obstacle->objectSpaceToWorldSpace(interaction_object_pos);
        Vec2f mouse_world_pos = Vec2f (mx / float(win_x), (win_y - my) / float(win_y));
        glVertex2f(object_world_pos[0], object_world_pos[1]);
        glVertex2f(mouse_world_pos[0], mouse_world_pos[1]);
        glEnd();
    }
}

/*
  ----------------------------------------------------------------------
   User interaction handeling
  ----------------------------------------------------------------------
*/

static void add_dens_and_vel_from_UI(float *d, float *u, float *v) {
    int i, j, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++) {
        u[i] = v[i] = d[i] = 0.0f;
    }

    if (!mouse_down[GLUT_LEFT_BUTTON] && !mouse_down[GLUT_RIGHT_BUTTON] || interacting_obstacle)
        return;

    i = (int) ((mx / (float) win_x) * N + 1);
    j = (int) (((win_y - my) / (float) win_y) * N + 1);

    if (i < 1 || i > N || j < 1 || j > N)
        return;

    if (mouse_down[GLUT_LEFT_BUTTON]) {
        u[IX(i, j)] = force * (mx - omx);
        v[IX(i, j)] = force * (omy - my);
    }

    if (mouse_down[GLUT_RIGHT_BUTTON]) {
        d[IX(i, j)] = source;
    }
}

// Debug helper funciton
static void print_obstacle_mask() {
    for (int x = 0; x < N + 4; x++)
        std::cout << "-";
    std::cout << "\n";

    for (int j = 0; j < N + 2; j++) {
        std::cout << "|";
        for (int i = 0; i < N + 2; i++) {
            std::cout << (obstacle_mask[IX(i, (N + 1) - j)] ? "█" : "x");
        }
        std::cout << "|\n";
    }

    for (int x = 0; x < N + 4; x++)
        std::cout << "-";
    std::cout << std::endl;
}

static void set_obstacle_mask() {
    for (int i = 0; i <= N + 1; i++) {
        for (int j = 0; j <= N + 1; j++) {
            obstacle_mask[IX(i, j)] = nullptr;
        }
    }
    for (int i = 0; i < obstacles.size(); i++) {
        obstacles[i]->addToObstacleMask(N, obstacle_mask);
    }
    // print_obstacle_mask();
}

static void handle_interaction() {
    if (!interacting_obstacle)
        return;

    Vec2f object_world_pos = interacting_obstacle->objectSpaceToWorldSpace(interaction_object_pos);
    Vec2f object_velocity = interacting_obstacle->getVelocityFromPosition(object_world_pos[0], object_world_pos[1]);
    Vec2f mouse_world_pos = Vec2f (mx / float(win_x), (win_y - my) / float(win_y));

    float spring_constant = 0.05;
    float damping = 0.02;
    Vec2f delta = (mouse_world_pos - object_world_pos);
    Vec2f force = spring_constant * delta - damping * object_velocity;

    interacting_obstacle->addForceAtPosition(force, object_world_pos);
}

static void begin_object_interaction() {
    float x = mx / (float) win_x;
    float y = ((win_y - my) / (float) win_y);

    for (auto o: obstacles) {
        if (o->isInside(x, y)) {
            interacting_obstacle = o;
            interaction_object_pos = o->worldSpaceToObjectSpace({x, y});
        }
    }
}

static void end_object_interaction() {
    if (!interacting_obstacle)
        return;

    set_obstacle_mask();
    interacting_obstacle = nullptr;
}

// Assumes only vertex-edge collisions
static void handle_collision() {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        for (size_t j = i + 1; j < obstacles.size(); ++j) {
            if (obstacles[i]->isCollidingWith(*obstacles[j]) || obstacles[j]->isCollidingWith(*obstacles[i])) { // o1 has vertex
                // Move back, ensures one vertex is inside other
                auto [collisionVertex, isFromObject1] = RectangleObstacle::bisection(dt, *obstacles[i], *obstacles[j]); // Point of collision is now in o1
                // Find vertex and edge normal
                Vec2f collisionNormal;
                if (isFromObject1) {
                    collisionNormal = obstacles[j]->getCollisionNormal(collisionVertex);
                } else {
                    collisionNormal = obstacles[i]->getCollisionNormal(collisionVertex);
                }

                // impulse function
                // Vec2f v1 = obstacles[i]->getVelocityFromPosition(collisionVertex[0], collisionVertex[1]);
                // Vec2f v2 = obstacles[j]->getVelocityFromPosition(collisionVertex[0], collisionVertex[1]);

            }
        }
    }
}

static void move_objects() {
    for (auto o: obstacles) {
        o->moveObject(dt);
    }
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

        case 'r':
        case 'R':
            vorticity_conf_enabled = !vorticity_conf_enabled;
            std::cout << "Vorticity confinement is " << (vorticity_conf_enabled ? "enabled." : "disabled.")
                      << std::endl;
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
    omy = my = y;

    mouse_down[button] = state == GLUT_DOWN;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
        begin_object_interaction();

    else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
        end_object_interaction();
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
    add_dens_and_vel_from_UI(dens_prev, u_prev, v_prev);
    handle_collision();
    handle_interaction();
    move_objects();
    set_obstacle_mask();
    vel_step(N, u, v, u_prev, v_prev, visc, dt, vorticity_conf_enabled ? vorticity_conf_epsilon : 0.0, obstacle_mask,
             obstacles);
    dens_step(N, dens, dens_prev, u, v, diff, dt, obstacle_mask, obstacles);

    omx = mx;
    omy = my;

    glutSetWindow(win_id);
    glutPostRedisplay();
}

static void display_func(void) {
    pre_display();

    if (dvel)
        draw_velocity();
    else
        draw_density();

    draw_obstacles();

    draw_interaction();

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

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
    printf("\t Toggle vorticity confinement with the 'r' key\n");

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
