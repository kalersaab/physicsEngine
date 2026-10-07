#include <cmath>
#include <iostream>
#include <stdexcept>

#include "kphysics/math/Vec3.h"
#include "kphysics/math/Quaternion.h"
#include "kphysics/math/Mat3.h"
#include "kphysics/physics/Inertia.h"
#include "kphysics/core/RigidBody.h"
#include "kphysics/core/PhysicsWorld.h"
#include "kphysics/physics/AABB.h"
#include "kphysics/physics/CollisionDetector.h"

using namespace kp;

namespace {

constexpr float EPSILON = 0.001f;

bool approx(float a, float b, float epsilon = EPSILON)
{
    return std::fabs(a - b) <= epsilon;
}

void expect(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testBoxInertia()
{
    std::cout << "[TEST] Box inertia...\n";

    const float mass = 2.0f;
    const float width = 2.0f;
    const float height = 4.0f;
    const float depth = 6.0f;

    Mat3 inertia =
        Inertia::box(mass, width, height, depth);

    expect(approx(inertia.m[0][0], 8.666666f),
           "Ixx incorrect");

    expect(approx(inertia.m[1][1], 6.666666f),
           "Iyy incorrect");

    expect(approx(inertia.m[2][2], 3.333333f),
           "Izz incorrect");

    std::cout << "  PASS\n";
}

void testWorldInverseInertia()
{
    std::cout << "[TEST] World-space inverse inertia...\n";

    RigidBody body;

    body.setMass(2.0f);

    Mat3 inertia =
        Inertia::box(
            2.0f,
            2.0f,
            4.0f,
            6.0f
        );

    body.setInertiaTensor(inertia);

    Quaternion rotation =
        Quaternion::fromAxisAngle(
            Vec3(0.0f, 1.0f, 0.0f),
            3.14159265359f / 2.0f
        );

    body.setOrientation(rotation);

    const Mat3& worldInverse =
        body.getWorldInverseInertiaTensor();

    const float expectedX = 1.0f / 3.333333f;
    const float expectedY = 1.0f / 6.666666f;
    const float expectedZ = 1.0f / 8.666666f;

    expect(
        approx(worldInverse.m[0][0], expectedX),
        "World I^-1 XX incorrect"
    );

    expect(
        approx(worldInverse.m[1][1], expectedY),
        "World I^-1 YY incorrect"
    );

    expect(
        approx(worldInverse.m[2][2], expectedZ),
        "World I^-1 ZZ incorrect"
    );

    std::cout << "  PASS\n";
}

void testGravity()
{
    std::cout << "[TEST] Gravity integration...\n";

    PhysicsWorld world;

    RigidBody* body = world.createBody();

    body->setMass(1.0f);
    body->setPosition(
        Vec3(0.0f, 10.0f, 0.0f)
    );

    world.step(1.0f);

    Vec3 position = body->getPosition();

    expect(
        approx(position.y, 0.19f, 0.01f),
        "Gravity integration incorrect"
    );

    std::cout << "  PASS\n";
}

void testStaticBody()
{
    std::cout << "[TEST] Static body...\n";

    PhysicsWorld world;

    RigidBody* body = world.createBody();

    body->setMass(0.0f);

    body->setPosition(
        Vec3(0.0f, 100.0f, 0.0f)
    );

    world.step(1.0f);

    Vec3 position = body->getPosition();

    expect(
        approx(position.y, 100.0f),
        "Static body moved"
    );

    std::cout << "  PASS\n";
}

void testFixedTimestep()
{
    std::cout << "[TEST] Fixed timestep...\n";

    PhysicsWorld world;

    world.setFixedTimeStep(
        1.0f / 60.0f
    );

    RigidBody* body = world.createBody();

    body->setMass(1.0f);

    body->setPosition(
        Vec3(0.0f, 10.0f, 0.0f)
    );

    for (int i = 0; i < 30; ++i) {
        world.update(1.0f / 30.0f);
    }

    Vec3 position = body->getPosition();

    expect(
        position.y < 5.5f,
        "Fixed timestep simulation incorrect"
    );

    expect(
        position.y > 3.0f,
        "Fixed timestep simulation unstable"
    );

    std::cout << "  PASS\n";
}
    void testAABB()
{
    std::cout << "[TEST] AABB intersection...\n";

    AABB a(
        Vec3(-1.0f, -1.0f, -1.0f),
        Vec3(1.0f, 1.0f, 1.0f)
    );

    AABB b(
        Vec3(0.5f, 0.5f, 0.5f),
        Vec3(2.0f, 2.0f, 2.0f)
    );

    AABB c(
        Vec3(3.0f, 3.0f, 3.0f),
        Vec3(4.0f, 4.0f, 4.0f)
    );

    expect(
        a.intersects(b),
        "AABBs should intersect"
    );

    expect(
        !a.intersects(c),
        "AABBs should not intersect"
    );

    expect(
        a.contains(Vec3(0.0f, 0.0f, 0.0f)),
        "Point should be inside AABB"
    );

    expect(
        !a.contains(Vec3(2.0f, 0.0f, 0.0f)),
        "Point should be outside AABB"
    );

    std::cout << "  PASS\n";
}
    void testBroadPhase()
{
    std::cout << "[TEST] Broad phase...\n";

    PhysicsWorld world;

    RigidBody* a = world.createBody();
    RigidBody* b = world.createBody();
    RigidBody* c = world.createBody();

    a->setPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    b->setPosition(
        Vec3(0.8f, 0.0f, 0.0f)
    );

    c->setPosition(
        Vec3(10.0f, 0.0f, 0.0f)
    );

    a->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    b->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    c->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    auto pairs = world.getCollisionPairs();

    expect(
        pairs.size() == 1,
        "Expected exactly one collision pair"
    );

    expect(
        (
            (pairs[0].first == a &&
             pairs[0].second == b)
            ||
            (pairs[0].first == b &&
             pairs[0].second == a)
        ),
        "Incorrect collision pair"
    );

    std::cout << "  PASS\n";
}

void testAABBCollision()
{
    std::cout << "[TEST] AABB collision detection...\n";

    PhysicsWorld world;

    RigidBody* a = world.createBody();
    RigidBody* b = world.createBody();

    a->setMass(1.0f);
    b->setMass(1.0f);

    a->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    b->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    a->setPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    b->setPosition(
        Vec3(1.5f, 0.0f, 0.0f)
    );

    Contact contact;

    bool collided =
        CollisionDetector::aabbVsAabb(
            *a,
            *b,
            contact
        );

    expect(
        collided,
        "Expected collision"
    );

    expect(
        approx(contact.penetration, 0.5f),
        "Incorrect penetration"
    );

    expect(
        approx(contact.normal.x, 1.0f),
        "Incorrect collision normal"
    );

    expect(
        approx(contact.normal.y, 0.0f),
        "Incorrect collision normal Y"
    );

    expect(
        approx(contact.normal.z, 0.0f),
        "Incorrect collision normal Z"
    );

    expect(
        approx(contact.point.x, 0.75f),
        "Incorrect contact point X"
    );

    std::cout << "  PASS\n";
}

    void testNoAABBCollision()
{
    std::cout << "[TEST] AABB no-collision detection...\n";

    PhysicsWorld world;

    RigidBody* a = world.createBody();
    RigidBody* b = world.createBody();

    a->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    b->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    a->setPosition(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    b->setPosition(
        Vec3(5.0f, 0.0f, 0.0f)
    );

    Contact contact;

    bool collided =
        CollisionDetector::aabbVsAabb(
            *a,
            *b,
            contact
        );

    expect(
        !collided,
        "Bodies should not collide"
    );

    std::cout << "  PASS\n";
}
    void testCollisionResponse()
{
    std::cout << "[TEST] Collision response...\n";

    PhysicsWorld world;

    world.setGravity(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    RigidBody* a = world.createBody();
    RigidBody* b = world.createBody();

    a->setMass(1.0f);
    b->setMass(1.0f);

    a->setRestitution(1.0f);
    b->setRestitution(1.0f);

    a->setFriction(0.0f);
    b->setFriction(0.0f);

    a->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    b->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    a->setPosition(
        Vec3(-0.9f, 0.0f, 0.0f)
    );

    b->setPosition(
        Vec3(0.9f, 0.0f, 0.0f)
    );


    a->setVelocity(
        Vec3(1.0f, 0.0f, 0.0f)
    );

    b->setVelocity(
        Vec3(-1.0f, 0.0f, 0.0f)
    );

    world.step(1.0f / 60.0f);

    expect(
        a->getVelocity().x < 0.0f,
        "Body A did not bounce"
    );

    expect(
        b->getVelocity().x > 0.0f,
        "Body B did not bounce"
    );

    const float distance =
        std::fabs(
            b->getPosition().x -
            a->getPosition().x
        );

    expect(
        distance >= 1.94f,
        "Bodies were not separated"
    );

    std::cout << "  PASS\n";
}
    void testAngularCollisionResponse()
{
    std::cout << "[TEST] Angular collision response...\n";

    PhysicsWorld world;

    world.setGravity(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    RigidBody* a = world.createBody();
    RigidBody* b = world.createBody();

    a->setMass(2.0f);
    b->setMass(2.0f);

    a->setRestitution(0.5f);
    b->setRestitution(0.5f);

    a->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    b->setHalfExtents(
        Vec3(1.0f, 1.0f, 1.0f)
    );

    a->setPosition(
        Vec3(-1.5f, 0.0f, 0.0f)
    );

    b->setPosition(
        Vec3(0.5f, 0.5f, 0.0f)
    );


    a->setVelocity(
        Vec3(2.0f, 0.0f, 0.0f)
    );

    b->setVelocity(
        Vec3(0.0f, 0.0f, 0.0f)
    );

    const Vec3 before =
        a->getAngularVelocity();

    world.step(1.0f / 60.0f);

    const Vec3 after =
        a->getAngularVelocity();

    const float angularChange =
        (after - before).length();

    expect(
        angularChange > 0.0001f,
        "Off-center collision produced no rotation"
    );

    std::cout << "  PASS\n";
}
    void testFriction()
{
    std::cout << "[TEST] Friction...\n";

    PhysicsWorld world;

    world.setGravity(
        Vec3(0.0f, -9.81f, 0.0f)
    );

    // Ground
    RigidBody* ground =
        world.createBody();

    ground->setMass(0.0f);

    ground->setHalfExtents(
        Vec3(5.0f, 0.5f, 5.0f)
    );

    ground->setPosition(
        Vec3(0.0f, -0.5f, 0.0f)
    );

    ground->setFriction(0.8f);

    RigidBody* box =
        world.createBody();

    box->setMass(1.0f);

    box->setHalfExtents(
        Vec3(0.5f, 0.5f, 0.5f)
    );

    box->setPosition(
        Vec3(0.0f, 0.6f, 0.0f)
    );

    box->setVelocity(
        Vec3(5.0f, 0.0f, 0.0f)
    );

    box->setFriction(0.8f);

    const float initialSpeed =
        box->getVelocity().x;

    for (int i = 0; i < 120; ++i)
    {
        world.step(1.0f / 60.0f);
    }

    const float finalSpeed =
        std::fabs(box->getVelocity().x);

    expect(
        finalSpeed < initialSpeed,
        "Friction did not reduce horizontal velocity"
    );

    expect(
        box->getPosition().y > 0.3f,
        "Box fell through ground"
    );

    std::cout << "  PASS\n";
}
    void testRestingContact()
{
    PhysicsWorld world;

    world.setGravity(
        Vec3(0.0f, -9.81f, 0.0f)
    );

    // Ground
    RigidBody* ground =
        world.createBody();

    ground->setMass(0.0f);

    ground->setHalfExtents(
        Vec3(5.0f, 0.5f, 5.0f)
    );

    ground->setPosition(
        Vec3(0.0f, -0.5f, 0.0f)
    );

    ground->setRestitution(0.0f);
    ground->setFriction(0.8f);

    // Dynamic box
    RigidBody* box =
        world.createBody();

    box->setMass(1.0f);

    box->setHalfExtents(
        Vec3(0.5f, 0.5f, 0.5f)
    );

    box->setPosition(
        Vec3(0.0f, 3.0f, 0.0f)
    );

    box->setRestitution(0.2f);
    box->setFriction(0.8f);

    for (int i = 0; i < 300; ++i)
    {
        world.step(1.0f / 60.0f);
    }

    const Vec3 position =
        box->getPosition();

    const Vec3 velocity =
        box->getVelocity();

    std::cout
        << "  Final position: "
        << position.x << ", "
        << position.y << ", "
        << position.z
        << "\n";

    std::cout
        << "  Final velocity: "
        << velocity.x << ", "
        << velocity.y << ", "
        << velocity.z
        << "\n";

    // Box should remain close to the ground.
    expect(
        position.y > 0.45f &&
        position.y < 0.65f,
        "Resting box has unstable position"
    );

    expect(
        std::fabs(velocity.y) < 0.2f,
        "Resting box has excessive vertical velocity"
    );
}
}



int main()
{
    try {

        std::cout << "\n";
        std::cout << "=================================\n";
        std::cout << "      KALER PHYSICS TESTS\n";
        std::cout << "=================================\n\n";

        testBoxInertia();
        testWorldInverseInertia();
        testGravity();
        testStaticBody();
        testFixedTimestep();
        testAABB();
        testBroadPhase();
        testAABBCollision();
        testNoAABBCollision();
        testCollisionResponse();
        testAngularCollisionResponse();
        testFriction();
        testRestingContact();

        std::cout << "\n=================================\n";
        std::cout << "ALL TESTS PASSED\n";
        std::cout << "=================================\n\n";

        return 0;

    } catch (const std::exception& e) {

        std::cerr << "\nTEST FAILED:\n";
        std::cerr << e.what() << "\n";

        return 1;
    }
}