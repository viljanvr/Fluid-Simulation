// #pragma once
//
// #include <gfx/vec2.h>
// #include "Object.h"
//
// #define PI 3.1415926535897932384626433
//
// class SolidCircle : public Object {
// public:
//     SolidCircle(int N, int x, int y, int radius, float mass);
//     SolidCircle(int N, Vec2f position, float radius, float mass);
//     Vec2f getVelocityFromPosition(float x, float y) override;
//     Vec2f getCGVelocity() override;
//     void setVelocity(Vec2f velocity) override;
//     void moveObject(float dt) override;
//     void alignPositionToGrid(int N) override;
//     void addToObstacleMask(int N, Object **obstacle_mask) override;
//     void draw() override;
//     bool isInside(float x, float y) override;
//     void addForceAtPosition(Vec2f force, Vec2f position) override;
//     std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const override;
//
//     Vec2f m_Velocity;
//
// private:
//     float m_Radius;
// };
