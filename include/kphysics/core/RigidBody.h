#pragma once

#include "kphysics/math/Vec3.h"
#include "kphysics/math/Mat3.h"
#include "kphysics/math/Quaternion.h"
#include "kphysics/physics/AABB.h"

namespace kp {

    class RigidBody {
    public:
        RigidBody();

        void setMass(float mass);

        float getMass() const;
        float getInverseMass() const;

        void setPosition(const Vec3& position);

        const Vec3& getPosition() const;

        void setOrientation(const Quaternion& orientation);

        const Quaternion& getOrientation() const;

        const Vec3& getVelocity() const;

        const Vec3& getAngularVelocity() const;

        void applyForce(const Vec3& force);

        void applyForceAtPoint(
            const Vec3& force,
            const Vec3& point
        );

        void clearForces();

        void applyTorque(const Vec3& torque);

        void clearTorque();

        void setInertiaTensor(const Mat3& inertia);

        const Mat3& getInertiaTensor() const;

        const Mat3& getInverseInertiaTensor() const;
        const Mat3& getWorldInverseInertiaTensor() const;

        void integrate(float dt);
        void setHalfExtents(const Vec3& halfExtents);

        const Vec3& getHalfExtents() const;

        const AABB& getWorldAABB() const;

        void updateWorldAABB();
        void setRestitution(float restitution);
        float getRestitution() const;
        void setVelocity(const Vec3& velocity);
        void applyImpulse(
            const Vec3& impulse,
            const Vec3& contactVector
        );

        void setAngularVelocity(const Vec3 &angularVelocity);
        void setFriction(float friction);
        float getFriction() const;

    private:
        void updateWorldInverseInertia();

        Vec3 position_;
        Vec3 velocity_;
        Vec3 acceleration_;

        Vec3 force_;

        float mass_;
        float inverseMass_;

        Quaternion orientation_;

        Vec3 angularVelocity_;

        Vec3 torque_;

        Mat3 inertiaTensor_;
        Mat3 inverseInertiaTensor_;
        Mat3 worldInverseInertiaTensor_;
        Vec3 halfExtents_;
        AABB worldAABB_;
        float restitution_;
        float friction_;
    };

}