#include "kphysics/physics/BroadPhase.h"

namespace kp {

    void BroadPhase::computePairs(
        const std::vector<RigidBody*>& bodies,
        std::vector<CollisionPair>& pairs
    ) const
    {
        pairs.clear();

        const size_t count = bodies.size();

        for (size_t i = 0; i < count; ++i) {

            RigidBody* a = bodies[i];

            if (!a)
                continue;

            for (size_t j = i + 1; j < count; ++j) {

                RigidBody* b = bodies[j];

                if (!b)
                    continue;

                /*
                 * Two static bodies can never move
                 * because of a collision.
                 */
                if (
                    a->getInverseMass() == 0.0f &&
                    b->getInverseMass() == 0.0f
                ) {
                    continue;
                }

                if (
                    a->getWorldAABB().intersects(
                        b->getWorldAABB()
                    )
                ) {
                    pairs.emplace_back(a, b);
                }
            }
        }
    }

}