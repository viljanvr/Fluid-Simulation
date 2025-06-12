#pragma once

#include <array>
#include <gfx/vec2.h>
#include "Object.h"

class SolidRectangle : public Object {
public:
    SolidRectangle(int N, int x, int y, int w, int h, float mass);
    SolidRectangle(int N, Vec2f position, float width, float height, float mass);
    Vec2f getVelocityFromPosition(float x, float y) override;
    Vec2f getCGVelocity() override;
    Vec2f getWorldPosition(Vec2f relativePosition);
    std::array<float, 4> getBoundingBox();
    void setVelocity(Vec2f velocity) override;
    void moveObject(float dt) override;
    void alignPositionToGrid(int N) override;
    void addToObstacleMask(int N, Object **obstacle_mask) override;
    void draw() override;
    bool isInside(float x, float y) override;
    void addForceAtPosition(Vec2f force, Vec2f position) override;
    std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const override;

    Vec2f m_Velocity;

private:
    // In object space, i.e. relative to object center
    Vec2f m_P1;
    Vec2f m_P2;
};
