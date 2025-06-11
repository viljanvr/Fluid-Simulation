#include "SolidBoundary.h"

#define IX(i, j) ((i) + (N + 2) * (j))

SolidBoundary::SolidBoundary(int N) : m_N{N} {}

Vec2f SolidBoundary::getVelocityFromPosition(float x, float y) { return {0.0f, 0.0f}; }

Vec2f SolidBoundary::getCGVelocity() { return {0.0f, 0.0f}; }

void SolidBoundary::setVelocity(Vec2f vel) {}

void SolidBoundary::moveObject(float dt) {}

void SolidBoundary::alignPositionToGrid(int N) {};

void SolidBoundary::addToObstacleMask(int N, Object **obstacle_mask) {
    for (size_t i = 0; i < N + 1; i++) {
        obstacle_mask[IX(i + 1, 0)] = this;
        obstacle_mask[IX(i + 1, N + 1)] = this;
        obstacle_mask[IX(0, i + 1)] = this;
        obstacle_mask[IX(N + 1, i + 1)] = this;
    }
}

void SolidBoundary::draw() {}

bool SolidBoundary::isInside(float x, float y) { return false; }

std::optional<Vec2f> SolidBoundary::get_line_intersection(const Vec2f& start, const Vec2f& end) const {
    Vec2f dir = end - start;
    if (norm(end) < 1e-6) {
        return std::nullopt;
    }
    double t_min = 1.0;
    for (int i = 0; i < 2; i++) {
        double t1 = (0.5 - start[i] / dir[i]);
        double t2 = (m_N + 0.5 - start[i] / dir[i]);
        if (t1 >= 0) {
            t_min = std::min(t_min, t1);
        }
        if (t2 >= 0) {
            t_min = std::min(t_min, t2);
        }
    }
    if (t_min >= 1.0) {
        return std::nullopt;
    }

    return start + dir * t_min;
}



