#include "SolidRectangle.h"
#include <algorithm>
#include <cmath>
#include "gfx/vec2.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#define IX(i, j) ((i) + (N + 2) * (j))


SolidRectangle::SolidRectangle(int N, Vec2f position, float width, float height) :
    Object(position, Vec2f(0.0,0.0)), m_P1(Vec2f(-0.5 * width, -0.5 * height)), m_P2(Vec2f(0.5 * width, 0.5 * height)) {
    alignPositionToGrid(N);
}

// x, y, w, h in gridcells
SolidRectangle::SolidRectangle(int N, int x, int y, int w, int h) :
    Object(Vec2f((float) x / N, (float) y / N), Vec2f(0.0,0.0)),
        m_P1(Vec2f(-0.5 * (float) w / N, -0.5 * (float) h / N)),
        m_P2(Vec2f(0.5 * (float) w / N, 0.5 * (float) h / N)) {}

Vec2f SolidRectangle::getVelocityFromPosition(float x, float y) { return m_Velocity; }

Vec2f SolidRectangle::getCGVelocity() { return m_Velocity; }

void SolidRectangle::addToObstacleMask(int N, Object **obstacle_mask) {
    int x1 = std::max(0, (int) std::round((m_Position[0] + m_P1[0]) * N));
    int x2 = std::min(N, (int) std::round((m_Position[0] + m_P2[0]) * N));
    int y1 = std::max(0, (int) std::round((m_Position[1] + m_P1[1]) * N));
    int y2 = std::min(N, (int) std::round((m_Position[1] + m_P2[1]) * N));

    for (int i = x1; i < x2; i++) {
        for (int j = y1; j < y2; j++) {
            obstacle_mask[IX(i + 1, j + 1)] = this;
        }
    }
}

void SolidRectangle::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void SolidRectangle::moveObject(float dt) {
    m_Position += m_Velocity * dt;
}

bool SolidRectangle::isInside(float x, float y) { return x >= m_Position[0] + m_P1[0] && y >= m_Position[1] + m_P1[1] &&
        x <= m_Position[0] + m_P2[0] && y <= m_Position[1] + m_P2[1]; }

void SolidRectangle::alignPositionToGrid(int N) {
    m_Position[0] = std::round(m_Position[0] * N) / N;
    m_Position[1] = std::round(m_Position[1] * N) / N;
}

void SolidRectangle::draw() {
    glBegin(GL_QUADS);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glVertex2f(m_Position[0] + m_P1[0], m_Position[1] + m_P1[1]); // bottom left
    glVertex2f(m_Position[0] + m_P2[0], m_Position[1] + m_P1[1]); // bottom right
    glVertex2f(m_Position[0] + m_P2[0], m_Position[1] + m_P2[1]); // top right
    glVertex2f(m_Position[0] + m_P1[0], m_Position[1] + m_P2[1]); // top left
    glEnd();
}
