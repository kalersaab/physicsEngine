#pragma once

#include <utility>
#include <vector>

#include "kphysics/core/RigidBody.h"

namespace kp {

    class BroadPhase {
    public:
        using CollisionPair =
            std::pair<RigidBody*, RigidBody*>;

        void computePairs(
            const std::vector<RigidBody*>& bodies,
            std::vector<CollisionPair>& pairs
        ) const;
    };

}