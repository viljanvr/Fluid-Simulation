#pragma once

#include <gfx/vec2.h>

class SolidObject {
public:
    SolidObject(int N, int x1, int y1, int x2, int y2);
    SolidObject(int N, Vec2f p1, Vec2f p2);
    Vec2f getVelocityFromPosition(float x, float y);
    Vec2f getCGVelocity();
    void setVelocity(Vec2f velocity);
    void moveObject();
    void alignPositionToGrid(int N);
    void addToObstacleMask(int N, SolidObject **obstacle_mask);
    void draw();
    bool isInside(float x, float y);

    Vec2f m_Velocity;

private:
    // Obstacle is defined by to diagonal corners P1, P2
    Vec2f m_P1;
    Vec2f m_P2;
};
