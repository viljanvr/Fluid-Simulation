#pragma once
#include "Object.h"

class SolidBoundary : public Object {
public:
    SolidBoundary(int N);
    Vec2f getVelocityFromPosition(float x, float y) override;
    Vec2f getCGVelocity() override;
    void setVelocity(Vec2f velocity) override;
    void moveObject(float dt) override;
    void alignPositionToGrid(int N) override;
    void addToObstacleMask(int N, Object **obstacle_mask) override;
    void draw() override;
    bool isInside(float x, float y) override;
    std::optional<Vec2f> get_line_intersection(const Vec2f& start, const Vec2f& end) const override;
    void addForceAtPosition(Vec2f force, Vec2f position) override;
private:
    int m_N;
};
