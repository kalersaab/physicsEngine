#pragma once

#include <memory>
#include <vector>

#include "kphysics/core/RigidBody.h"
#include "kphysics/math/Vec3.h"

namespace kp {

    class PhysicsWorld {
    public:
        PhysicsWorld();

        RigidBody* createBody();

        void setGravity(const Vec3& gravity);

        void step(float dt);

        const std::vector<std::unique_ptr<RigidBody>>&
        getBodies() const;

    private:
        Vec3 gravity_;

        std::vector<std::unique_ptr<RigidBody>> bodies_;
    };

}