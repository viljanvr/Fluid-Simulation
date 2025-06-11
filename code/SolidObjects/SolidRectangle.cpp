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


SolidRectangle::SolidRectangle(int N, Vec2f p1, Vec2f p2) : m_P1(p1), m_P2(p2), m_Velocity(0.0, 0.0) {
    alignPositionToGrid(N);
}

SolidRectangle::SolidRectangle(int N, int x1, int y1, int x2, int y2) :
    m_P1(Vec2f((float) x1 / N, (float) y1 / N)), m_P2(Vec2f((float) x2 / N, (float) y2 / N)), m_Velocity(0.0, 0.0) {}

Vec2f SolidRectangle::getVelocityFromPosition(float x, float y) { return m_Velocity; }

Vec2f SolidRectangle::getCGVelocity() { return m_Velocity; }

void SolidRectangle::addToObstacleMask(int N, Object **obstacle_mask) {
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

void SolidRectangle::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void SolidRectangle::moveObject(float dt) {
    m_P1 += m_Velocity * dt;
    m_P2 += m_Velocity * dt;
}

bool SolidRectangle::isInside(float x, float y) { return x >= m_P1[0] && y >= m_P1[1] && x <= m_P2[0] && y <= m_P2[1]; }

void SolidRectangle::alignPositionToGrid(int N) {
    m_P1[0] = std::round(m_P1[0] * N) / N;
    m_P1[1] = std::round(m_P1[1] * N) / N;
    m_P2[0] = std::round(m_P2[0] * N) / N;
    m_P2[1] = std::round(m_P2[1] * N) / N;
}

void SolidRectangle::draw() {
    glBegin(GL_QUADS);
    glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
    glVertex2f((m_P1[0]), (m_P1[1])); // bottom left
    glVertex2f((m_P2[0]), (m_P1[1])); // bottom right
    glVertex2f((m_P2[0]), (m_P2[1])); // top right
    glVertex2f((m_P1[0]), (m_P2[1])); // top left
    glEnd();
}

std::optional<Vec2f> SolidRectangle::get_line_intersection(const Vec2f& start, const Vec2f& end) const {
    if (norm(start - end) < 1e-6) {
        return std::nullopt;
    }

    // TODO with rb: remove redundant computation of cg and local coordinates of widht / height
    Vec2f cg = (m_P1 + m_P2) / 2.0;
    Vec2f size = m_P2 - m_P1;

    // TODO with rb: update conversion start and end to local coordinates
    Vec2f local_start = start - cg;
    Vec2f local_end = end - cg;
    Vec2f dir = local_end - local_start;

    double tmin = 0.0;
    double tmax = 1.0;
    // Iterate over x and y
    for (int i = 0; i < 2; i++) {
        // Parallel to slab
        if (std::abs(dir[i]) < 1e-8) {
            if (local_start[i] < -size[i] / 2 || local_start[i] > size[i] / 2) {
                return std::nullopt;
            }
        } else {
            double t1 = (-size[i] / 2 - local_start[i]) / dir[i];
            double t2 = (size[i] / 2 - local_start[i]) / dir[i];
            if (t1 > t2) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax) {
                return std::nullopt;
            }
        }
    }

    Vec2f local_intersection = local_start + tmin * dir;

    //TODO with rb: update conversion to world space coordinates
    Vec2f intersection = local_intersection + cg;

    return intersection;
}
