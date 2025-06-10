#pragma once
#include <gfx/vec2.h>

class Object {
public:
    Object(const Vec2f& position, const Vec2f& velocity) :
        m_Position(position), m_Velocity(velocity) {}
    virtual Vec2f getVelocityFromPosition(float x, float y) = 0;
    virtual Vec2f getCGVelocity() = 0;
    virtual void setVelocity(Vec2f velocity) = 0;
    virtual void moveObject(float dt) = 0;
    virtual void alignPositionToGrid(int N) = 0;
    virtual void addToObstacleMask(int N, Object **obstacle_mask) = 0;
    virtual void draw() = 0;
    virtual bool isInside(float x, float y) = 0;

protected:
    Vec2f m_Position;
    Vec2f m_Velocity;
};
