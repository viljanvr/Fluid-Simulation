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
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "RectangleObstacle.h"
#include "ScenePresets.h"
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
extern void temp_step(int N, float *x, float *x0, float *u, float *v, float diff, float dt,
                      RectangleObstacle **obstacle_mask, const std::vector<RectangleObstacle *> &obstacle_list);
extern void vel_step(int N, float *u, float *v, float *u0, float *v0, float *temp, float visc, float dt,
                     float vorticity_conf_epsilon, bool pressure_force_enabled, RectangleObstacle **obstacle_mask,
                     const std::vector<RectangleObstacle *> &obstacle_list);


/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;
static float vorticity_conf_epsilon = 160;
static bool vorticity_conf_enabled = true;
static bool temp_enabled = false;
static bool pressure_force_enabled = true;
static InteractionMode current_interaction_mode = RIGID;
static bool collision_enabled = true;
static bool should_sprinkle_density_once = false;

static float *u, *v, *u_prev, *v_prev;
static float *dens, *dens_prev;
static float *temp, *temp_prev;

static int win_id;
static int win_x, win_y; // Window with in pixels
static int mouse_down[3];
static int omx, omy, mx, my; // Pixel-coordinate

static float (*colormap)[3] = jet_colormap;

static RectangleObstacle **obstacle_mask;
static std::vector<RectangleObstacle *> obstacles;
static RectangleObstacle *interacting_obstacle;
static Vec2f interaction_object_pos;

static bool ui_info_enabled = true;
static std::string ui_info = ""; // Toggle visibility with 'i'
static std::string ui_notification = "";
static int n_iterations = 0;
static std::chrono::high_resolution_clock::time_point start_time;

static int scene_id = 5;

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
    if (temp)
        free(temp);
    if (temp_prev)
        free(temp_prev);
    for (auto o: obstacles) {
        free(o);
    }
}

static void clear_data(void) {
    int i, size = (N + 2) * (N + 2);

    for (i = 0; i < size; i++) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
        temp[i] = temp_prev[i] = 0.0f;
        obstacle_mask[i] = nullptr;
    }

    for (auto o: obstacles) {
        delete o;
    }
    obstacles.clear();

    n_iterations = 0;
    start_time = std::chrono::high_resolution_clock::now();
}

static int allocate_data(void) {
    int size = (N + 2) * (N + 2);

    u = (float *) malloc(size * sizeof(float));
    v = (float *) malloc(size * sizeof(float));
    u_prev = (float *) malloc(size * sizeof(float));
    v_prev = (float *) malloc(size * sizeof(float));
    dens = (float *) malloc(size * sizeof(float));
    dens_prev = (float *) malloc(size * sizeof(float));
    temp = (float *) malloc(size * sizeof(float));
    temp_prev = (float *) malloc(size * sizeof(float));
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

static void color_from_dens_and_temp(float dens, float temp) {
    float r = std::clamp((temp + 20.0f) / 120.0f, 0.0f, 1.0f);
    // float g = std::clamp(1.0f - std::abs(temp - 50.0f) / 50.0f, 0.0f, 1.0f);
    float g = 0.3;
    float b = std::clamp(1.0f - temp / 120.0f, 0.0f, 1.0f);
    r *= std::clamp(dens, 0.0f, 1.0f) * 0.85 + 0.15;
    g *= std::clamp(dens, 0.0f, 1.0f) * 0.85 + 0.15;
    b *= std::clamp(dens, 0.0f, 1.0f) * 0.85 + 0.15;
    glColor3f(r, g, b);
}

static void draw_density_and_temp(void) {
    int i, j;
    float x, y, h, d00, d01, d10, d11, t00, t01, t10, t11;

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
            t00 = temp[IX(i, j)];
            t01 = temp[IX(i, j + 1)];
            t10 = temp[IX(i + 1, j)];
            t11 = temp[IX(i + 1, j + 1)];

            color_from_dens_and_temp(d00, t00);
            glVertex2f(x, y);
            color_from_dens_and_temp(d10, t10);
            glVertex2f(x + h, y);
            color_from_dens_and_temp(d11, t11);
            glVertex2f(x + h, y + h);
            color_from_dens_and_temp(d01, t01);
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
        Vec2f mouse_world_pos = Vec2f(mx / float(win_x), (win_y - my) / float(win_y));
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

static void add_dens_and_vel_from_UI(float *d, float *u, float *v, float *temp) {
    int i, j, size = (N + 2) * (N + 2);

    if (should_sprinkle_density_once) {
        should_sprinkle_density_once = false;
        int dots = size / 100;
        for (i = 0; i < dots; i++) {
            int x = (rand() % N) + 1;
            int y = (rand() % N) + 1;
            dens[IX(x, y)] = source / 10;
        }
    }

    for (i = 0; i < size; i++) {
        u[i] = v[i] = d[i] = temp[i] = 0.0f;
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
    switch (current_interaction_mode) {
        case RIGID: {
            Vec2f object_world_pos = interacting_obstacle->objectSpaceToWorldSpace(interaction_object_pos);
            Vec2f object_velocity =
                    interacting_obstacle->getVelocityFromPosition(object_world_pos[0], object_world_pos[1]);
            Vec2f mouse_world_pos = Vec2f(mx / float(win_x), (win_y - my) / float(win_y));

            float spring_constant = 0.05;
            float damping = 0.02;
            Vec2f delta = (mouse_world_pos - object_world_pos);
            Vec2f force = spring_constant * delta - damping * object_velocity;

            interacting_obstacle->addForceAtPosition(force, object_world_pos);
            break;
        }
        case SOLID: {
            float delta_x = (mx - omx) / (float) win_x;
            float delta_y = (omy - my) / (float) win_y;
            interacting_obstacle->setVelocity(Vec2f(delta_x, delta_y) / dt);
            interacting_obstacle->m_AngularVelocity = 0.0f;
        }
        default:
            break;
    }
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

// Assumes only vertex-edge collisions, single point of contact
static void handle_collision() {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        for (size_t j = i + 1; j < obstacles.size(); ++j) {
            for (auto vertex: obstacles[i]->getVerticesInRectangle(*obstacles[j])) {
                RectangleObstacle::applyCollisionImpulse(vertex, *obstacles[i], *obstacles[j], dt);
            }
            for (auto vertex: obstacles[j]->getVerticesInRectangle(*obstacles[i])) {
                RectangleObstacle::applyCollisionImpulse(vertex, *obstacles[j], *obstacles[i], dt);
            }
            // if (obstacles[i]->isCollidingWith(*obstacles[j]) || obstacles[j]->isCollidingWith(*obstacles[i])) {
            //     // Backtrack
            //     auto [collisionVertex, isFromObject1] = RectangleObstacle::bisection(dt, *obstacles[i],
            //     *obstacles[j]);

            //    if (isFromObject1) {
            //        RectangleObstacle::applyCollisionImpulse(collisionVertex, *obstacles[i], *obstacles[j], dt);
            //    } else {
            //        RectangleObstacle::applyCollisionImpulse(collisionVertex, *obstacles[j], *obstacles[i], dt);
            //    }
            //}
        }
        for (auto vertex: obstacles[i]->getVerticesInWall({0.0f, 0.0f}, {1.0f, 0.0f})) {
            obstacles[i]->applyWallCollisionImpulse(vertex, {1.0f, 0.0f}, dt);
        }
        for (auto vertex: obstacles[i]->getVerticesInWall({0.0f, 0.0f}, {0.0f, 1.0f})) {
            obstacles[i]->applyWallCollisionImpulse(vertex, {0.0f, 1.0f}, dt);
        }
        for (auto vertex: obstacles[i]->getVerticesInWall({1.0f, 1.0f}, {-1.0f, 0.0f})) {
            obstacles[i]->applyWallCollisionImpulse(vertex, {-1.0f, 0.0f}, dt);
        }
        for (auto vertex: obstacles[i]->getVerticesInWall({1.0f, 1.0f}, {0.0f, -1.0f})) {
            obstacles[i]->applyWallCollisionImpulse(vertex, {0.0f, -1.0f}, dt);
        }
    }
}


static void move_objects() {
    for (auto o: obstacles) {
        if (current_interaction_mode == SOLID) {
            o->m_AngularVelocity = 0.0f;
        }
        o->moveObject(dt);
        if (current_interaction_mode == SOLID) {
            o->m_Velocity = Vec2f(0.0f, 0.0f);
        }
    }
}


/*
  ----------------------------------------------------------------------
   Text rendering
  ----------------------------------------------------------------------
*/

static int getBitmapStringWidth(const std::string &text, void *font) {
    int width = 0;
    for (char c: text) {
        width += glutBitmapWidth(font, c);
    }
    return width;
}

std::vector<std::string> splitLines(const std::string &str) {
    std::vector<std::string> lines;
    std::istringstream ss(str);
    std::string line;
    while (std::getline(ss, line)) {
        lines.push_back(line);
    }
    return lines;
}

static void draw_text(float x, float y, std::string &str, void *font = GLUT_BITMAP_HELVETICA_12,
                      std::array<float, 3> color = {1.0, 1.0, 1.0}) {
    if (str == "")
        return;

    float line_height = 0.03;
    auto lines = splitLines(str);

    glColor3f(color[0], color[1], color[2]);
    for (size_t i = 0; i < lines.size(); ++i) {
        glRasterPos2f(x, y - i * line_height);
        for (char c: lines[i]) {
            glutBitmapCharacter(font, c);
        }
    }
}

static void update_info_text() {

    auto now = std::chrono::high_resolution_clock::now();

    auto elapsed_time = std::chrono::duration<double>(now - start_time).count();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "Interaction mode (m): " << (current_interaction_mode == RIGID ? "Rigid body" : "Solid obstacle")
       << "\nVorticity conf (r): " << (vorticity_conf_enabled ? "ON" : "OFF")
       << "\nTemperature (t): " << (temp_enabled ? "ON" : "OFF")
       << "\nPressure force (p): " << (pressure_force_enabled ? "ON" : "OFF")
       << "\nCollisions (x): " << (collision_enabled ? "ON" : "OFF") << "\n\nIterations: " << n_iterations
       << "\nAvg. itererations/s: " << n_iterations / elapsed_time;
    ui_info = ss.str();
}

static void clear_notification(int value) { ui_notification = ""; }

static void set_notification(std::string message, int duration = 3000) {
    ui_notification = message;
    glutTimerFunc(duration, clear_notification, 0);
}

static void draw_ui_overlay() {
    if (ui_info_enabled) {
        update_info_text();
        draw_text(0.05, 0.95, ui_info);
    }

    float centered_x = 0.5 * (1 - static_cast<float>(getBitmapStringWidth(ui_notification, GLUT_BITMAP_HELVETICA_12)) /
                                          static_cast<float>(win_x));
    draw_text(centered_x, 0.05, ui_notification, GLUT_BITMAP_HELVETICA_12, {1.0, 0.0, 0.0});
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y) {
    std::ostringstream ss;

    switch (key) {
        case 'c':
        case 'C':
            clear_data();
            ScenePresets::setScene(N, scene_id, obstacles, vorticity_conf_enabled, collision_enabled,
                                   pressure_force_enabled, temp_enabled, current_interaction_mode);
            break;

        case 'q':
        case 'Q':
            free_data();
            exit(0);
            break;

        case 'r':
        case 'R':
            vorticity_conf_enabled = !vorticity_conf_enabled;
            ss << "Vorticity confinement is " << (vorticity_conf_enabled ? "enabled." : "disabled.");
            std::cout << ss.str() << std::endl;
            set_notification(ss.str());
            break;
        case 't':
        case 'T':
            temp_enabled = !temp_enabled;
            ss << "Temperature is " << (temp_enabled ? "enabled." : "disabled.");
            std::cout << ss.str() << std::endl;
            set_notification(ss.str());

            if (!temp_enabled) {
                for (int i = 0; i < (N + 2) * (N + 2); i++) {
                    temp[i] = temp_prev[i] = 0.0f;
                }
            }
            break;
        case 'p':
        case 'P':
            pressure_force_enabled = !pressure_force_enabled;
            ss << "Pressure force is " << (pressure_force_enabled ? "enabled." : "disabled.");
            std::cout << ss.str() << std::endl;
            set_notification(ss.str());
            break;
        case 'm':
        case 'M':
            current_interaction_mode = (InteractionMode) ((int) current_interaction_mode + 1) == Last
                                               ? (InteractionMode) 0
                                               : (InteractionMode) ((int) current_interaction_mode + 1);
            pressure_force_enabled = current_interaction_mode == RIGID;
            collision_enabled = current_interaction_mode == RIGID;
            ss << "Switched to " << (current_interaction_mode == RIGID ? "rigid body" : "solid obstacle") << " mode.";
            std::cout << ss.str() << std::endl;
            set_notification(ss.str());
            break;
        case 'v':
        case 'V':
            dvel = !dvel;
            break;
        case 'i':
        case 'I':
            ui_info_enabled = !ui_info_enabled;
            break;
        case 'x':
        case 'X':
            collision_enabled = !collision_enabled;
            ss << "Collisions are " << (collision_enabled ? "enabled." : "disabled.");
            std::cout << ss.str() << std::endl;
            set_notification(ss.str());
            break;
        case ' ':
            should_sprinkle_density_once = true;
            break;
        default:
            if (std::isdigit(key)) {
                scene_id = key - '0';
                clear_data();
                ScenePresets::setScene(N, scene_id, obstacles, vorticity_conf_enabled, collision_enabled,
                                       pressure_force_enabled, temp_enabled, current_interaction_mode);
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
    add_dens_and_vel_from_UI(dens_prev, u_prev, v_prev, temp_prev);
    handle_interaction();
    set_obstacle_mask();
    vel_step(N, u, v, u_prev, v_prev, temp, visc, dt, vorticity_conf_enabled ? vorticity_conf_epsilon : 0.0,
             pressure_force_enabled, obstacle_mask, obstacles);
    dens_step(N, dens, dens_prev, u, v, diff, dt, obstacle_mask, obstacles);
    if (temp_enabled) {
        temp_step(N, temp, temp_prev, u, v, 0.00001, dt, obstacle_mask, obstacles);
    }
    if (collision_enabled) {
        handle_collision();
    }
    move_objects();

    omx = mx;
    omy = my;

    n_iterations++;
    glutSetWindow(win_id);
    glutPostRedisplay();
}

static void display_func(void) {
    pre_display();

    if (dvel)
        draw_velocity();
    else
        draw_density_and_temp();

    draw_obstacles();

    draw_interaction();

    draw_ui_overlay();

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
    printf("\t Add densities with the right mouse button.\n");
    printf("\t Sprinkle density over large area with space.\n");
    printf("\t Add velocities with the left mouse button and dragging the mouse\n");
    printf("\t Toggle density/velocity display with the 'v' key\n");
    printf("\t Clear the simulation by pressing the 'c' key\n");
    printf("\t Quit by pressing the 'q' key\n");
    printf("\t Toggle simulation info with the 'i' key\n");
    printf("\t Change the interaction mode with the 'm' key\n");
    printf("\t Toggle vorticity confinement with the 'r' key\n");
    printf("\t Toggle temperature with the 't' key\n");
    printf("\t Toggle pressure forces with the 'p' key\n");
    printf("\t Toggle collision with the 'x' key\n");

    dvel = 0;

    if (!allocate_data())
        exit(1);
    clear_data();
    ScenePresets::setScene(N, scene_id, obstacles, vorticity_conf_enabled, collision_enabled, pressure_force_enabled,
                           temp_enabled, current_interaction_mode);

    win_x = 512;
    win_y = 512;
    open_glut_window();

    start_time = std::chrono::high_resolution_clock::now();

    glutMainLoop();

    exit(0);
}
