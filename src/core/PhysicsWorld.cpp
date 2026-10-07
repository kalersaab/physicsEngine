#include "kphysics/core/PhysicsWorld.h"
#include "kphysics/physics/CollisionDetector.h"
namespace kp {

    PhysicsWorld::PhysicsWorld()
    : gravity_(0.0f, -9.81f, 0.0f),
      fixedTimeStep_(1.0f / 60.0f),
      accumulator_(0.0f) {
    }

    RigidBody* PhysicsWorld::createBody() {

        auto body = std::make_unique<RigidBody>();

        RigidBody* ptr = body.get();

        bodies_.push_back(std::move(body));

        return ptr;
    }

    void PhysicsWorld::setGravity(const Vec3& gravity) {
        gravity_ = gravity;
    }

    void PhysicsWorld::step(float dt)
    {
        for (auto& body : bodies_)
        {
            if (body->getInverseMass() <= 0.0f)
                continue;

            body->applyForce(
                gravity_ *
                body->getMass()
            );
        }

        for (auto& body : bodies_)
        {
            body->integrate(dt);
        }

        for (auto& body : bodies_)
        {
            body->updateWorldAABB();
        }

        std::vector<Contact> contacts =
            detectCollisions();
        
        impulseSolver_.solve(
            contacts,
            dt
        );
    }

    void PhysicsWorld::update(float frameTime) {

        accumulator_ += frameTime;

        while (accumulator_ >= fixedTimeStep_) {

            step(fixedTimeStep_);

            accumulator_ -= fixedTimeStep_;
        }
    }

    void PhysicsWorld::setFixedTimeStep(
        float timestep
    ) {
        if (timestep <= 0.0f) {
            return;
        }

        fixedTimeStep_ = timestep;
    }

    float PhysicsWorld::getFixedTimeStep() const {
        return fixedTimeStep_;
    }
    const std::vector<std::unique_ptr<RigidBody>>&
    PhysicsWorld::getBodies() const {
        return bodies_;
    }
    std::vector<BroadPhase::CollisionPair>
    PhysicsWorld::getCollisionPairs() const
    {
        std::vector<RigidBody*> bodies;

        bodies.reserve(bodies_.size());

        for (const auto& body : bodies_) {
            bodies.push_back(body.get());
        }

        std::vector<BroadPhase::CollisionPair> pairs;

        broadPhase_.computePairs(
            bodies,
            pairs
        );

        return pairs;
    }

    std::vector<Contact>
    PhysicsWorld::detectCollisions() const
    {
        std::vector<Contact> contacts;

        auto pairs = getCollisionPairs();

        for (const auto& pair : pairs)
        {
            Contact contact;

            if (
                CollisionDetector::aabbVsAabb(
                    *pair.first,
                    *pair.second,
                    contact
                )
            )
            {
                contact.bodyA = pair.first;
                contact.bodyB = pair.second;

                contacts.push_back(contact);
            }
        }

        return contacts;
    }

}