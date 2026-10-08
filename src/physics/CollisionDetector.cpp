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
bool CollisionDetector::aabbVsAabb(
    const RigidBody& a,
    const RigidBody& b,
    ContactManifold& manifold)
{
    manifold.clear();

    const AABB& aabbA = a.getWorldAABB();
    const AABB& aabbB = b.getWorldAABB();

    const Vec3 minA = aabbA.getMin();
    const Vec3 maxA = aabbA.getMax();

    const Vec3 minB = aabbB.getMin();
    const Vec3 maxB = aabbB.getMax();

    const float overlapX =
        std::min(maxA.x, maxB.x) -
        std::max(minA.x, minB.x);

    const float overlapY =
        std::min(maxA.y, maxB.y) -
        std::max(minA.y, minB.y);

    const float overlapZ =
        std::min(maxA.z, maxB.z) -
        std::max(minA.z, minB.z);

    if (overlapX < 0.0f ||
        overlapY < 0.0f ||
        overlapZ < 0.0f)
    {
        return false;
    }

    float penetration = overlapX;
    int axis = 0;

    if (overlapY < penetration)
    {
        penetration = overlapY;
        axis = 1;
    }

    if (overlapZ < penetration)
    {
        penetration = overlapZ;
        axis = 2;
    }

    const Vec3 centerA = aabbA.center();
    const Vec3 centerB = aabbB.center();

    Vec3 normal;

    if (axis == 0)
    {
        normal =
            centerB.x >= centerA.x
                ? Vec3(1, 0, 0)
                : Vec3(-1, 0, 0);
    }
    else if (axis == 1)
    {
        normal =
            centerB.y >= centerA.y
                ? Vec3(0, 1, 0)
                : Vec3(0, -1, 0);
    }
    else
    {
        normal =
            centerB.z >= centerA.z
                ? Vec3(0, 0, 1)
                : Vec3(0, 0, -1);
    }

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

    RigidBody* bodyA =
        const_cast<RigidBody*>(&a);

    RigidBody* bodyB =
        const_cast<RigidBody*>(&b);

    if (axis == 0)
    {
        const float x =
            normal.x > 0.0f
                ? maxA.x
                : minA.x;

        const float y0 = overlapMin.y;
        const float y1 = overlapMax.y;

        const float z0 = overlapMin.z;
        const float z1 = overlapMax.z;

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x, y0, z0),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x, y0, z1),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x, y1, z0),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x, y1, z1),
            normal,
            penetration
        });
    }
    else if (axis == 1)
    {
        const float y =
            normal.y > 0.0f
                ? maxA.y
                : minA.y;

        const float x0 = overlapMin.x;
        const float x1 = overlapMax.x;

        const float z0 = overlapMin.z;
        const float z1 = overlapMax.z;

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x0, y, z0),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x0, y, z1),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x1, y, z0),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x1, y, z1),
            normal,
            penetration
        });
    }
    else
    {
        const float z =
            normal.z > 0.0f
                ? maxA.z
                : minA.z;

        const float x0 = overlapMin.x;
        const float x1 = overlapMax.x;

        const float y0 = overlapMin.y;
        const float y1 = overlapMax.y;

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x0, y0, z),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x0, y1, z),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x1, y0, z),
            normal,
            penetration
        });

        manifold.addContact({
            bodyA,
            bodyB,
            Vec3(x1, y1, z),
            normal,
            penetration
        });
    }

    return manifold.getContactCount() > 0;
}

}