#pragma once
#include <gfx/vec2.h>
#include <optional>

class Object {
public:
    virtual Vec2f getVelocityFromPosition(float x, float y) = 0;
    virtual Vec2f getCGVelocity() = 0;
    virtual void setVelocity(Vec2f velocity) = 0;
    virtual void moveObject(float dt) = 0;
    virtual void alignPositionToGrid(int N) = 0;
    virtual void addToObstacleMask(int N, Object **obstacle_mask) = 0;
    virtual void draw() = 0;
    virtual bool isInside(float x, float y) = 0;
    virtual std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const = 0;
};
