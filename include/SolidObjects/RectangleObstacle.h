#pragma once

#include <array>
#include <gfx/vec2.h>
#include <optional>

class RectangleObstacle {
public:
    RectangleObstacle(int N, int x, int y, int w, int h, float density);
    RectangleObstacle(int N, Vec2f position, float width, float height, float density);
    Vec2f getVelocityFromPosition(float x, float y);
    Vec2f getCGVelocity();
    std::array<float, 4> getBoundingBox();
    std::array<Vec2f, 4> getWorldSpaceVertices() const;
    void setVelocity(Vec2f velocity);
    void moveObject(float dt);
    void alignPositionToGrid(int N);
    void addToObstacleMask(int N, RectangleObstacle **obstacle_mask);
    void draw();
    bool isInside(float x, float y) const;
    void addForceAtPosition(Vec2f force, Vec2f position);
    std::optional<Vec2f> get_line_intersection(const Vec2f &start, const Vec2f &end) const;
    Vec2f worldSpaceToObjectSpace(const Vec2f &position) const;
    Vec2f objectSpaceToWorldSpace(const Vec2f &position) const;
    Vec2f getCollisionNormal(const Vec2f &position) const;
    // static std::optional<Vec2f> areColliding(const RectangleObstacle& o1, const RectangleObstacle& o2);
    std::optional<Vec2f> isCollidingWith(const RectangleObstacle &other) const;

    static std::pair<Vec2f, bool> bisection(float dt, RectangleObstacle &o1, RectangleObstacle &o2);
    static void applyCollisionImpulse(Vec2f collisionVertex, RectangleObstacle &vertexObj, RectangleObstacle &faceObj);

    Vec2f m_Velocity = Vec2f(0.0, 0.0);
    float m_AngularVelocity = 0.0f;

private:
    Vec2f m_P1; // Relative bottom left position from m_Position
    Vec2f m_P2; // Relative top right position from m_Position
    Vec2f m_Position; // Position in world space
    Vec2f m_Force = Vec2f(0.0, 0.0);
    float m_Mass;
    float m_Torque = 0.0f;
    float m_Rotation = 0.0f;
    float m_Inertia;
};
