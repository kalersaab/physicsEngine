#pragma once

#include "kphysics/core/RigidBody.h"
#include "kphysics/physics/Contact.h"
#include "kphysics/physics/ContactManifold.h"

namespace kp {

    class CollisionDetector {
    public:
        static bool aabbVsAabb(
            const RigidBody& a,
            const RigidBody& b,
            Contact& contact
        );

        static bool aabbVsAabb(
            const RigidBody& a,
            const RigidBody& b,
            ContactManifold& manifold
        );
    };

}