#pragma once
#include <gfx/vec2.h>

class Object {
public:
    virtual Vec2f getVelocityFromPosition(float x, float y) = 0;
    virtual Vec2f getCGVelocity() = 0;
    virtual void setVelocity(Vec2f velocity) = 0;
    virtual void moveObject() = 0;
    virtual void alignPositionToGrid(int N) = 0;
    virtual void addToObstacleMask(int N, Object **obstacle_mask) = 0;
    virtual void draw() = 0;
    virtual bool isInside(float x, float y) = 0;
};
