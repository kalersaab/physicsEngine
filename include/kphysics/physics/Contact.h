#pragma once

#include "kphysics/math/Vec3.h"

namespace kp {

class RigidBody;

struct Contact {
    RigidBody* bodyA;
    RigidBody* bodyB;

    Vec3 point;
    Vec3 normal;

    float penetration;

    Contact()
        : bodyA(nullptr),
          bodyB(nullptr),
          point(0, 0, 0),
          normal(0, 1, 0),
          penetration(0.0f)
    {
    }

    Contact(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec3& point,
        const Vec3& normal,
        float penetration
    )
        : bodyA(bodyA),
          bodyB(bodyB),
          point(point),
          normal(normal),
          penetration(penetration)
    {
    }
};

}