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


SolidRectangle::SolidRectangle(int N, Vec2f position, float width, float height, float mass) :
    Object(position, Vec2f(0.0,0.0), mass), m_P1(Vec2f(-0.5 * width,-0.5 * height)),
        m_P2(Vec2f(0.5 * width, 0.5 * height)) {

    m_Inertia = m_Mass * (4 * m_P2[0] * m_P2[0] + 4 * m_P2[1] * m_P2[1]) / 12;
    alignPositionToGrid(N);
}

// x, y, w, h in grid cells
SolidRectangle::SolidRectangle(int N, int x, int y, int w, int h, float mass) :
    Object(Vec2f((float) x / N, (float) y / N), Vec2f(0.0,0.0), mass),
        m_P1(Vec2f(-0.5 * (float) w / N, -0.5 * (float) h / N)),
        m_P2(Vec2f(0.5 * (float) w / N, 0.5 * (float) h / N)) {}

Vec2f SolidRectangle::getVelocityFromPosition(float x, float y) { return m_Velocity; }

Vec2f SolidRectangle::getCGVelocity() { return m_Velocity; }

void SolidRectangle::addToObstacleMask(int N, Object **obstacle_mask) {
    std::array<float, 4> bb = getBoundingBox();
    int minX = std::max(0, (int) std::round(bb[0] * N));
    int maxX = std::min(N, (int) std::round(bb[1] * N));
    int minY = std::max(0, (int) std::round(bb[2] * N));
    int maxY = std::min(N, (int) std::round(bb[3] * N));

    for (int i = minX; i < maxX; i++) {
        for (int j = minY; j < maxY; j++) {

            obstacle_mask[IX(i + 1, j + 1)] = this;
        }
    }
}

void SolidRectangle::addForceAtPosition(Vec2f force, Vec2f position) {
    Vec2f relative = position - m_Position;
    float torqueFactor = 0.0005;
    m_Torque += torqueFactor * (relative[0] * force[1] - relative[1] * force[0]);
    m_Force += force;    // Needs to be addition when we add collision
}

void SolidRectangle::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void SolidRectangle::moveObject(float dt) {
    m_Velocity += (m_Force / m_Mass) * dt;
    m_AngularVelocity += (m_Torque / m_Inertia) * dt;

    m_Position += m_Velocity * dt;
    m_Rotation = std::fmod(m_Rotation + m_AngularVelocity * dt, 360.0f);

    // Reset for next iteration
    m_Force = 0;
    m_Torque = 0;
}

// Uses bounding box of the rectangle
bool SolidRectangle::isInside(float x, float y) {
    std::array<float, 4> bb = getBoundingBox();
    return x >= bb[0] && x <= bb[1] && y >= bb[2] && y <= bb[3];
}

void SolidRectangle::alignPositionToGrid(int N) {
    m_Position[0] = std::round(m_Position[0] * N) / N;
    m_Position[1] = std::round(m_Position[1] * N) / N;
}

void SolidRectangle::draw() {
    glBegin(GL_QUADS);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    // Calculate corner positions including rotation
    Vec2f botLeft = getWorldPosition(m_P1);
    Vec2f botRight = getWorldPosition(Vec2f(m_P2[0], m_P1[1]));
    Vec2f topLeft = getWorldPosition(Vec2f(m_P1[0], m_P2[1]));
    Vec2f topRight = getWorldPosition(m_P2);

    // Render world positions of the corners
    glVertex2f(botLeft[0], botLeft[1]);
    glVertex2f(botRight[0], botRight[1]);
    glVertex2f(topRight[0], topRight[1]);
    glVertex2f(topLeft[0], topLeft[1]);
    glEnd();
}

Vec2f SolidRectangle::getWorldPosition(Vec2f relativePos) {
    float cosRot = std::cos(m_Rotation);
    float sinRot = std::sin(m_Rotation);
    Vec2f rotation (relativePos[0] * cosRot - relativePos[1] * sinRot,
                        relativePos[0] * sinRot + relativePos[1] * cosRot);
    return rotation + m_Position;
}

std::array<float, 4> SolidRectangle::getBoundingBox() {
    Vec2f botLeft = getWorldPosition(m_P1);
    Vec2f botRight = getWorldPosition(Vec2f(m_P2[0], m_P1[1]));
    Vec2f topLeft = getWorldPosition(Vec2f(m_P1[0], m_P2[1]));
    Vec2f topRight = getWorldPosition(m_P2);
    float minX = std::min({botLeft[0], botRight[0], topLeft[0], topRight[0]});
    float maxX = std::max({botLeft[0], botRight[0], topLeft[0], topRight[0]});
    float minY = std::min({botLeft[1], botRight[1], topLeft[1], topRight[1]});
    float maxY = std::max({botLeft[1], botRight[1], topLeft[1], topRight[1]});

    return {minX, maxX, minY, maxY};
}
