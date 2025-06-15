#include "RectangleObstacle.h"
#include <algorithm>
#include <cmath>
#include "gfx/vec2.h"

#if defined(__APPLE__) && defined(__aarch64__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


#define IX(i, j) ((i) + (N + 2) * (j))


RectangleObstacle::RectangleObstacle(int N, Vec2f position, float width, float height, float density) :
    m_Position(position), m_Mass(density * width * height), m_P1(Vec2f(-0.5 * width, -0.5 * height)),
    m_P2(Vec2f(0.5 * width, 0.5 * height)) {

    m_Inertia = m_Mass * (4 * m_P2[0] * m_P2[0] + 4 * m_P2[1] * m_P2[1]) / 12;
    alignPositionToGrid(N);
}

// x, y, w, h in grid cells
RectangleObstacle::RectangleObstacle(int N, int i, int j, int w, int h, float density) :
    RectangleObstacle(N, Vec2f(((float) i + 0.5f) / N, ((float) j + 0.5f) / N), (float) w / N, (float) h / N, density) {}

Vec2f RectangleObstacle::getVelocityFromPosition(float x, float y) {
    Vec2f r = Vec2f(x, y) - m_Position;
    Vec2 rPerp(-r[1], r[0]);
    Vec2f rotationalVelocity = m_AngularVelocity * rPerp;
    return rotationalVelocity + m_Velocity;
}

Vec2f RectangleObstacle::getCGVelocity() { return m_Velocity; }

void RectangleObstacle::addToObstacleMask(int N, RectangleObstacle **obstacle_mask) {
    std::array<float, 4> bb = getBoundingBox();
    int minI = std::max(0, (int) std::round(bb[0] * N));
    int maxI = std::min(N, (int) std::round(bb[1] * N));
    int minJ = std::max(0, (int) std::round(bb[2] * N));
    int maxJ = std::min(N, (int) std::round(bb[3] * N));

    for (int i = minI; i < maxI; i++) {
        for (int j = minJ; j < maxJ; j++) {
            float x = ((float) i + 0.5) / N;
            float y = ((float) j + 0.5) / N;
            if (isInside(x, y)) {
                obstacle_mask[IX(i + 1, j + 1)] = this;
            }
        }
    }
}

void RectangleObstacle::addForceAtPosition(Vec2f force, Vec2f position) {
    Vec2f relative = position - m_Position;
    float torqueFactor = 1.0;
    m_Torque += torqueFactor * (relative[0] * force[1] - relative[1] * force[0]);
    m_Force += force; // Needs to be addition when we add collision
}

void RectangleObstacle::setVelocity(Vec2f velocity) { m_Velocity = velocity; }

void RectangleObstacle::moveObject(float dt) {
    m_Velocity += (m_Force / m_Mass) * dt;
    m_AngularVelocity += (m_Torque / m_Inertia) * dt;

    m_Position += m_Velocity * dt;
    m_Rotation = std::fmod(m_Rotation + m_AngularVelocity * dt, 360.0f);

    // Reset for next iteration
    m_Force = 0;
    m_Torque = 0;
}

bool RectangleObstacle::isInside(float x, float y) const {
    Vec2f objectSpace = worldSpaceToObjectSpace(Vec2f(x, y));
    return objectSpace[0] >= m_P1[0] && objectSpace[1] >= m_P1[1] && objectSpace[0] <= m_P2[0] &&
                objectSpace[1] <= m_P2[1];
}

void RectangleObstacle::alignPositionToGrid(int N) {
    m_Position[0] = std::round(m_Position[0] * N) / N;
    m_Position[1] = std::round(m_Position[1] * N) / N;
}

void RectangleObstacle::draw() {
    glBegin(GL_QUADS);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    // Calculate corner positions including rotation
    Vec2f botLeft = objectSpaceToWorldSpace(m_P1);
    Vec2f botRight = objectSpaceToWorldSpace(Vec2f(m_P2[0], m_P1[1]));
    Vec2f topLeft = objectSpaceToWorldSpace(Vec2f(m_P1[0], m_P2[1]));
    Vec2f topRight = objectSpaceToWorldSpace(m_P2);

    // Render world positions of the corners
    glVertex2f(botLeft[0], botLeft[1]);
    glVertex2f(botRight[0], botRight[1]);
    glVertex2f(topRight[0], topRight[1]);
    glVertex2f(topLeft[0], topLeft[1]);
    glEnd();
}

std::array<float, 4> RectangleObstacle::getBoundingBox() {
    Vec2f botLeft = objectSpaceToWorldSpace(m_P1);
    Vec2f botRight = objectSpaceToWorldSpace(Vec2f(m_P2[0], m_P1[1]));
    Vec2f topLeft = objectSpaceToWorldSpace(Vec2f(m_P1[0], m_P2[1]));
    Vec2f topRight = objectSpaceToWorldSpace(m_P2);
    float minX = std::min({botLeft[0], botRight[0], topLeft[0], topRight[0]});
    float maxX = std::max({botLeft[0], botRight[0], topLeft[0], topRight[0]});
    float minY = std::min({botLeft[1], botRight[1], topLeft[1], topRight[1]});
    float maxY = std::max({botLeft[1], botRight[1], topLeft[1], topRight[1]});

    return {minX, maxX, minY, maxY};
}

std::optional<Vec2f> RectangleObstacle::get_line_intersection(const Vec2f& start, const Vec2f& end) const {
    if (norm(start - end) < 1e-6) {
        return std::nullopt;
    }

    Vec2f local_start = worldSpaceToObjectSpace(start);
    Vec2f local_end = worldSpaceToObjectSpace(end);
    Vec2f dir = local_end - local_start;

    double tmin = 0.0;
    double tmax = 1.0;
    // Iterate over x and y
    for (int i = 0; i < 2; i++) {
        // Parallel to slab
        if (std::abs(dir[i]) < 1e-8) {
            if (local_start[i] < m_P1[i] || local_start[i] > m_P2[i]) {
                return std::nullopt;
            }
        } else {
            if (std::abs(dir[i]) < 1e-6) {
                continue;
            }
            double t1 = (m_P1[i] - local_start[i]) / dir[i];
            double t2 = (m_P2[i] - local_start[i]) / dir[i];
            if (t1 > t2) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax) {
                return std::nullopt;
            }
        }
    }

    Vec2f local_intersection = local_start + tmin * dir;

    Vec2f intersection = objectSpaceToWorldSpace(local_intersection);

    return intersection;
}

Vec2f RectangleObstacle::worldSpaceToObjectSpace(const Vec2f& position) const {
    Vec2f delta = position - m_Position;
    return Vec2f(std::cos(m_Rotation) * delta[0] + std::sin(m_Rotation) * delta[1],
            -std::sin(m_Rotation) * delta[0] + std::cos(m_Rotation) * delta[1]);
}

Vec2f RectangleObstacle::objectSpaceToWorldSpace(const Vec2f& position) const {
    float cosRot = std::cos(m_Rotation);
    float sinRot = std::sin(m_Rotation);
    Vec2f rotation (position[0] * cosRot - position[1] * sinRot,
                        position[0] * sinRot + position[1] * cosRot);
    return rotation + m_Position;
}

Vec2f closestPointOnEdge(const Vec2f& A, const Vec2f& B, const Vec2f& position) {
    Vec2f AB = B - A;
    float t = ((position - A) * AB) / (AB * AB);
    t = std::clamp(t, 0.0f, 1.0f);
    return A + t * AB;
}

// Finds the closest edge, and computes the normal on it
Vec2f RectangleObstacle::getCollisionNormal(const Vec2f& position) const{
    auto vertices = getWorldSpaceVertices();
    float minDist = std::numeric_limits<float>::max();
    Vec2f closestEdge;

    for (int i = 0; i < 4; ++i) {
        Vec2f A = vertices[i];
        Vec2f B = vertices[(i + 1) % 4];

        Vec2f closest = closestPointOnEdge(A, B, position);
        float dist = norm(position - closest);

        if (dist < minDist) {
            minDist = dist;
            closestEdge = B - A;
        }
    }
    Vec2f collisionNormal (closestEdge[1], -closestEdge[0]);
    // std::cout << "collisionNormal: " << collisionNormal / norm(collisionNormal) << std::endl;
    return collisionNormal / norm(collisionNormal);
}

void RectangleObstacle::applyCollisionImpulse(Vec2f collisionVertex, RectangleObstacle &vertexObj, RectangleObstacle &faceObj) {
    float epsilon = 0.1;

    // Get normal on faceObj
    Vec2f collisionNormal = faceObj.getCollisionNormal(collisionVertex);

    // sub-computations for j
    Vec2f v1 = vertexObj.getVelocityFromPosition(collisionVertex[0], collisionVertex[1]);
    Vec2f v2 = faceObj.getVelocityFromPosition(collisionVertex[0], collisionVertex[1]);
    float vRelNeg = collisionNormal * (v1 - v2);

    // If this is positive, the obstacles are moving away from each other and we don't want to apply forces.
    if (vRelNeg > 0) {
       return;
    }

    Vec2f r1 = collisionVertex - vertexObj.m_Position;
    Vec2f r2 = collisionVertex - faceObj.m_Position;
    float r1Cross = r1[0] * collisionNormal[1] - r1[1] * collisionNormal[0];
    float r2Cross = r2[0] * collisionNormal[1] - r2[1] * collisionNormal[0];
    float denom1 = (r1Cross * r1Cross) / vertexObj.m_Inertia;
    float denom2 = (r2Cross * r2Cross) / faceObj.m_Inertia;

    // Final j computation
    float numerator = -(1 + epsilon) * vRelNeg;
    float denominator = 1 / vertexObj.m_Mass + 1 / faceObj.m_Mass + denom1 + denom2;
    float j = numerator / denominator;
    Vec2f force = collisionNormal * j;

    // std::cout << "force: " << force << std::endl;
    // std::cout << "angular pre: " << vertexObj.m_AngularVelocity << std::endl;

    // Add to velocity states
    vertexObj.m_Velocity += force / vertexObj.m_Mass;
    faceObj.m_Velocity -= force / faceObj.m_Mass;
    vertexObj.m_AngularVelocity += (r1[0] * force[1] - r1[1] * force[0]) / vertexObj.m_Inertia;
    faceObj.m_AngularVelocity -= (r2[0] * force[1] - r2[1] * force[0]) / faceObj.m_Inertia;
    // std::cout << "angular post: " << vertexObj.m_AngularVelocity << std::endl;
}


std::optional<Vec2f> RectangleObstacle::isCollidingWith(const RectangleObstacle& other) const {
    std::array<Vec2f, 4> vertices = getWorldSpaceVertices();
    for (const auto& v : vertices) {
        if (other.isInside(v[0], v[1])) {
            return v;
        }
    }
    return std::nullopt;
}

// Modifies object positions to approximated collision positions
std::pair<Vec2f, bool> RectangleObstacle::bisection(float dt, RectangleObstacle& o1, RectangleObstacle& o2) {
    float eps = 0.000001; // error we accept from collision time t
    float t0 = 0;
    float t = dt;
    Vec2f o1Start = o1.m_Position;
    Vec2f o2Start = o2.m_Position;
    Vec2f lastCollidingPos1 = o1.m_Position;
    Vec2f lastCollidingPos2 = o2.m_Position;
    bool isFromObject1 = false;

    while (t - t0 > eps) {
        float tMid = 0.5 * (t0 + t);
        o1.m_Position = o1Start - tMid * o1.m_Velocity;
        o2.m_Position = o2Start - tMid * o2.m_Velocity;
        if (o1.isCollidingWith(o2) || o2.isCollidingWith(o1)) {
            if (o1.isCollidingWith(o2)) {
                lastCollidingPos1 = o1.isCollidingWith(o2).value();
                isFromObject1 = true;
            } else {
                lastCollidingPos2 = o2.isCollidingWith(o1).value();
                isFromObject1 = false;
            }
            t = tMid;
        } else {
            t0 = tMid;
        }
    }

    if (isFromObject1) return {lastCollidingPos1, isFromObject1};
    return {lastCollidingPos2, isFromObject1};
}

std::array<Vec2f, 4> RectangleObstacle::getWorldSpaceVertices() const {
    return {
        objectSpaceToWorldSpace(m_P1), // bottom-left
        objectSpaceToWorldSpace(Vec2f(m_P2[0], m_P1[1])), // bottom-right
        objectSpaceToWorldSpace(m_P2), // top-right
        objectSpaceToWorldSpace(Vec2f(m_P1[0], m_P2[1]))  // top-left
    };
}