#include "SolidObject.h"
#include <algorithm>
#include <cmath>
#include "gfx/vec2.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#define IX(i, j) ((i) + (N + 2) * (j))


SolidObject::SolidObject(int N, Vec2f p1, Vec2f p2) : m_P1(p1), m_P2(p2), m_Velocity(0.0, 0.0) {
    alignPositionToGrid(N);
}

SolidObject::SolidObject(int N, int x1, int y1, int x2, int y2) :
    m_P1(Vec2f((float) x1 / N, (float) y1 / N)), m_P2(Vec2f((float) x2 / N, (float) y2 / N)), m_Velocity(0.0, 0.0) {}

Vec2f SolidObject::getVelocityFromPosition(float x, float y) { return m_Velocity; }

Vec2f SolidObject::getCGVelocity() { return m_Velocity; }

void SolidObject::addToObstacleMask(int N, SolidObject **obstacle_mask) {
    int x1 = std::max(0, (int) std::round(m_P1[0] * N));
    int x2 = std::min(N, (int) std::round(m_P2[0] * N));
    int y1 = std::max(0, (int) std::round(m_P1[1] * N));
    int y2 = std::min(N, (int) std::round(m_P2[1] * N));

    for (int i = x1; i < x2; i++) {
        for (int j = y1; j < y2; j++) {
            obstacle_mask[IX(i + 1, j + 1)] = this;
        }
    }
}

void SolidObject::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void SolidObject::moveObject() {
    m_P1 += m_Velocity;
    m_P2 += m_Velocity;
}

bool SolidObject::isInside(float x, float y) { return x >= m_P1[0] && y >= m_P1[1] && x <= m_P2[0] && y <= m_P2[1]; }

void SolidObject::alignPositionToGrid(int N) {
    m_P1[0] = std::round(m_P1[0] * N) / N;
    m_P1[1] = std::round(m_P1[1] * N) / N;
    m_P2[0] = std::round(m_P2[0] * N) / N;
    m_P2[1] = std::round(m_P2[1] * N) / N;
}

void SolidObject::draw() {
    glBegin(GL_QUADS);
    glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
    glVertex2f((m_P1[0]), (m_P1[1])); // bottom left
    glVertex2f((m_P2[0]), (m_P1[1])); // bottom right
    glVertex2f((m_P2[0]), (m_P2[1])); // top right
    glVertex2f((m_P1[0]), (m_P2[1])); // top left
    glEnd();
}
