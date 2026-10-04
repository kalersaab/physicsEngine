#include "kphysics/core/PhysicsWorld.h"

namespace kp {

    PhysicsWorld::PhysicsWorld()
        : gravity_(0.0f, -9.81f, 0.0f) {
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

    void PhysicsWorld::step(float dt) {

        for (auto& body : bodies_) {

            if (body->getInverseMass() == 0.0f) {
                continue;
            }

            Vec3 gravityForce =
                gravity_ * body->getMass();

            body->applyForce(gravityForce);

            body->integrate(dt);
        }
    }

    const std::vector<std::unique_ptr<RigidBody>>&
    PhysicsWorld::getBodies() const {
        return bodies_;
    }

}