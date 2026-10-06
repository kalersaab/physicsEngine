#include "kphysics/physics/CollisionDetector.h"

#include <algorithm>
#include <cmath>

namespace kp {

bool CollisionDetector::aabbVsAabb(
    const RigidBody& a,
    const RigidBody& b,
    Contact& contact
)
{
    const AABB& aabbA = a.getWorldAABB();
    const AABB& aabbB = b.getWorldAABB();

    const Vec3& minA = aabbA.getMin();
    const Vec3& maxA = aabbA.getMax();

    const Vec3& minB = aabbB.getMin();
    const Vec3& maxB = aabbB.getMax();

    // Calculate overlap on each axis.
    const float overlapX =
        std::min(maxA.x, maxB.x) -
        std::max(minA.x, minB.x);

    const float overlapY =
        std::min(maxA.y, maxB.y) -
        std::max(minA.y, minB.y);

    const float overlapZ =
        std::min(maxA.z, maxB.z) -
        std::max(minA.z, minB.z);

    // No intersection.
    if (overlapX < 0.0f ||
        overlapY < 0.0f ||
        overlapZ < 0.0f)
    {
        return false;
    }

    /*
     * The smallest overlap is the Minimum
     * Translation Vector direction.
     *
     * We use it as the collision normal.
     */
    float penetration = overlapX;

    Vec3 normal;

    if (aabbB.center().x >= aabbA.center().x)
        normal = Vec3(1.0f, 0.0f, 0.0f);
    else
        normal = Vec3(-1.0f, 0.0f, 0.0f);

    if (overlapY < penetration)
    {
        penetration = overlapY;

        if (aabbB.center().y >= aabbA.center().y)
            normal = Vec3(0.0f, 1.0f, 0.0f);
        else
            normal = Vec3(0.0f, -1.0f, 0.0f);
    }

    if (overlapZ < penetration)
    {
        penetration = overlapZ;

        if (aabbB.center().z >= aabbA.center().z)
            normal = Vec3(0.0f, 0.0f, 1.0f);
        else
            normal = Vec3(0.0f, 0.0f, -1.0f);
    }

    /*
     * Approximate contact point:
     *
     * midpoint of the overlapping region.
     */
    const Vec3 overlapMin(
        std::max(minA.x, minB.x),
        std::max(minA.y, minB.y),
        std::max(minA.z, minB.z)
    );

    const Vec3 overlapMax(
        std::min(maxA.x, maxB.x),
        std::min(maxA.y, maxB.y),
        std::min(maxA.z, maxB.z)
    );

    contact.point =
        (overlapMin + overlapMax) * 0.5f;

    contact.normal = normal;
    contact.penetration = penetration;

    return true;
}

}