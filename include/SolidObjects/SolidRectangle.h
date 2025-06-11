#pragma once

#include <gfx/vec2.h>
#include "Object.h"

class SolidRectangle : public Object {
public:
    SolidRectangle(int N, int x1, int y1, int x2, int y2);
    SolidRectangle(int N, Vec2f p1, Vec2f p2);
    Vec2f getVelocityFromPosition(float x, float y) override;
    Vec2f getCGVelocity() override;
    void setVelocity(Vec2f velocity) override;
    void moveObject(float dt) override;
    void alignPositionToGrid(int N) override;
    void addToObstacleMask(int N, Object **obstacle_mask) override;
    void draw() override;
    bool isInside(float x, float y) override;
    std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const override;

    Vec2f m_Velocity;

private:
    // Obstacle is defined by to diagonal corners P1, P2
    Vec2f m_P1;
    Vec2f m_P2;
};
