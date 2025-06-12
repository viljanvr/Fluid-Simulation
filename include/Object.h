#pragma once
#include <gfx/vec2.h>
#include <optional>

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
    virtual std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const = 0;
    virtual void addForceAtPosition(Vec2f force, Vec2f position) = 0;
    Vec2f worldSpaceToObjectSpace(const Vec2f& position) {
        Vec2f delta = position - m_Position;
        return Vec2f(std::cos(m_Rotation) * delta[0] + std::sin(m_Rotation) * delta[1],
                -std::sin(m_Rotation) * delta[0] + std::cos(m_Rotation) * delta[1]);
    };
    Vec2f objectSpaceToWorldSpace(const Vec2f& position) {
        float cosRot = std::cos(m_Rotation);
        float sinRot = std::sin(m_Rotation);
        Vec2f rotation (position[0] * cosRot - position[1] * sinRot,
                            position[0] * sinRot + position[1] * cosRot);
        return rotation + m_Position;
    }

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
