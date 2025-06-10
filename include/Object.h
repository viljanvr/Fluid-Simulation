#pragma once
#include <gfx/vec2.h>

class Object {
public:
    Object(const Vec2f& position, const Vec2f& velocity, const float mass) :
        m_Position(position), m_Velocity(velocity), m_Mass(mass) {}
    virtual Vec2f getVelocityFromPosition(float x, float y) = 0;
    virtual Vec2f getCGVelocity() = 0;
    virtual void setVelocity(Vec2f velocity) = 0;
    virtual void moveObject(float dt) = 0;
    virtual void alignPositionToGrid(int N) = 0;
    virtual void addToObstacleMask(int N, Object **obstacle_mask) = 0;
    virtual void draw() = 0;
    virtual bool isInside(float x, float y) = 0;
    virtual void addForceAtPosition(Vec2f force, Vec2f position) = 0;

protected:
    Vec2f m_Position;
    Vec2f m_Velocity;
    Vec2f m_Force = Vec2f(0.0,0.0);
    float m_Mass;
    float m_Torque = 0.0f;
    float m_Rotation = 0.0f;
    float m_AngularVelocity = 0.0f;
    float m_Inertia;
};
