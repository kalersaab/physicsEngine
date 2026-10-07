#pragma once

#include "kphysics/core/RigidBody.h"
#include "kphysics/physics/Contact.h"

namespace kp {

    class CollisionDetector {
    public:
        static bool aabbVsAabb(
            const RigidBody& a,
            const RigidBody& b,
            Contact& contact
        );
    };

}