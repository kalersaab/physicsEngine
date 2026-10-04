#pragma once

#include "kphysics/math/Vec3.h"
#include "kphysics/math/Mat3.h"
#include "kphysics/math/Quaternion.h"

namespace kp {

    class RigidBody {
    public:
        RigidBody();

        // Mass
        void setMass(float mass);

        float getMass() const;
        float getInverseMass() const;

        // Position
        void setPosition(const Vec3& position);

        const Vec3& getPosition() const;

        // Orientation
        void setOrientation(const Quaternion& orientation);

        const Quaternion& getOrientation() const;

        // Linear velocity
        const Vec3& getVelocity() const;

        // Angular velocity
        const Vec3& getAngularVelocity() const;

        // Forces
        void applyForce(const Vec3& force);

        void applyForceAtPoint(
            const Vec3& force,
            const Vec3& point
        );

        void clearForces();

        // Torque
        void applyTorque(const Vec3& torque);

        void clearTorque();

        // Inertia
        void setInertiaTensor(const Mat3& inertia);

        const Mat3& getInertiaTensor() const;

        const Mat3& getInverseInertiaTensor() const;

        // Physics integration
        void integrate(float dt);

    private:

        // Linear state
        Vec3 position_;
        Vec3 velocity_;
        Vec3 acceleration_;

        // Forces
        Vec3 force_;

        // Mass
        float mass_;
        float inverseMass_;

        // Angular state
        Quaternion orientation_;

        Vec3 angularVelocity_;

        Vec3 torque_;

        // Rotational inertia
        Mat3 inertiaTensor_;
        Mat3 inverseInertiaTensor_;
    };

}