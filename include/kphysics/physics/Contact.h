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
              point(0.0f, 0.0f, 0.0f),
              normal(0.0f, 1.0f, 0.0f),
              penetration(0.0f)
        {
        }
    };

}