#pragma once

#include <gfx/vec2.h>
#include "Object.h"

class SolidRectangle : public Object {
public:
    SolidRectangle(int N, int x, int y, int w, int h);
    SolidRectangle(int N, Vec2f position, float width, float height);
    Vec2f getVelocityFromPosition(float x, float y) override;
    Vec2f getCGVelocity() override;
    void setVelocity(Vec2f velocity) override;
    void moveObject(float dt) override;
    void alignPositionToGrid(int N) override;
    void addToObstacleMask(int N, Object **obstacle_mask) override;
    void draw() override;
    bool isInside(float x, float y) override;


private:
    // In object space, i.e. relative to position
    Vec2f m_P1;
    Vec2f m_P2;
};
