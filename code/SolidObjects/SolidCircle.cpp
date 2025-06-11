#include "SolidCircle.h"
#include <algorithm>
#include <cmath>
#include "gfx/vec2.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#define IX(i, j) ((i) + (N + 2) * (j))


SolidCircle::SolidCircle(int N, Vec2f position, float radius) :
    m_Position(position), m_Radius(radius), m_Velocity(0.0, 0.0) {
    alignPositionToGrid(N);
}

SolidCircle::SolidCircle(int N, int x, int y, int radius) :
    m_Position(Vec2f((float) x / N, (float) y / N)), m_Radius((float) radius / N), m_Velocity(0.0, 0.0) {}

Vec2f SolidCircle::getVelocityFromPosition(float x, float y) { return m_Velocity; }

Vec2f SolidCircle::getCGVelocity() { return m_Velocity; }

void SolidCircle::addToObstacleMask(int N, Object **obstacle_mask) {
    int x1 = std::max(0, (int) ((m_Position[0] - m_Radius) * N));
    int x2 = std::min(N, (int) ((m_Position[0] + m_Radius) * N) + 1);
    int y1 = std::max(0, (int) ((m_Position[1] - m_Radius) * N));
    int y2 = std::min(N, (int) ((m_Position[1] + m_Radius) * N) + 1);

    float h = 1.0 / N;

    for (int i = x1; i < x2; i++) {
        for (int j = y1; j < y2; j++) {

            float delta_i = ((float) i + 0.5) * h - m_Position[0];
            float delta_j = ((float) j + 0.5) * h - m_Position[1];

            if (delta_i * delta_i + delta_j * delta_j <= m_Radius * m_Radius)
                obstacle_mask[IX(i + 1, j + 1)] = this;
        }
    }
}

void SolidCircle::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void SolidCircle::moveObject(float dt) { m_Position += m_Velocity * dt; }

bool SolidCircle::isInside(float x, float y) {
    float delta_x = x - m_Position[0];
    float delta_y = y - m_Position[1];
    return delta_x * delta_x + delta_y * delta_y <= m_Radius * m_Radius;
}

void SolidCircle::alignPositionToGrid(int N) {
    m_Position[0] = std::round(m_Position[0] * N) / N;
    m_Position[1] = std::round(m_Position[1] * N) / N;
}

void SolidCircle::draw() {
    // Draw Circle
    // glBegin(GL_LINE_LOOP);
    // glColor3f(1.0, 0.0, 0.0);
    // for (int i = 0; i < 360; i = i + 18) {
    //     float degInRad = i * PI / 180;
    //     glVertex2f(m_Position[0] + cos(degInRad) * m_Radius, m_Position[1] + sin(degInRad) * m_Radius);
    // }
    // glEnd();

    glBegin(GL_POLYGON);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    for (int i = 0; i < 360; i = i + 18) {
        float degInRad = i * PI / 180;
        glVertex2f(m_Position[0] + cos(degInRad) * m_Radius, m_Position[1] + sin(degInRad) * m_Radius);
    }
    glEnd();
}

std::optional<Vec2f> SolidCircle::get_line_intersection(const Vec2f& start, const Vec2f& end) const {
    return std::nullopt;
}