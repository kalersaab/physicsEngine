#pragma once

#include <memory>
#include <vector>

#include "kphysics/core/RigidBody.h"
#include "kphysics/math/Vec3.h"
#include "kphysics/physics/BroadPhase.h"
#include "kphysics/physics/Contact.h"
#include "kphysics/physics/ImpulseSolver.h"
#include "kphysics/physics/ContactManifold.h"

namespace kp {

    class PhysicsWorld {
    public:
        PhysicsWorld();

        RigidBody* createBody();

        void setGravity(const Vec3& gravity);

        std::vector<Contact> detectCollisions() const;

        void step(float dt);

        void update(float frameTime);

        void setFixedTimeStep(float timestep);

        float getFixedTimeStep() const;

        const std::vector<std::unique_ptr<RigidBody>>&
        getBodies() const;
        std::vector<BroadPhase::CollisionPair>
        getCollisionPairs() const;
        
        std::vector<ContactManifold>
        detectContactManifolds() const;

    private:
        BroadPhase broadPhase_;

    private:
        Vec3 gravity_;

        std::vector<std::unique_ptr<RigidBody>> bodies_;

        float fixedTimeStep_;
        ImpulseSolver impulseSolver_;
        float accumulator_;
    };

}