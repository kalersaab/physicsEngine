#include <iostream>

#include "kphysics/core/RigidBody.h"

int main() {

    kp::RigidBody body;

    body.setMass(1.0f);

    body.setPosition(
        kp::Vec3(0.0f, 0.0f, 0.0f)
    );

    // Unit inertia tensor for now
    body.setInertiaTensor(
        kp::Mat3::identity()
    );

    // Apply torque around Y
    body.applyTorque(
        kp::Vec3(0.0f, 10.0f, 0.0f)
    );

    constexpr float dt = 1.0f / 60.0f;

    for (int frame = 0; frame < 60; ++frame) {

        body.integrate(dt);

        const auto& q =
            body.getOrientation();

        const auto& angularVelocity =
            body.getAngularVelocity();

        std::cout
            << "Frame: " << frame
            << " | Quaternion: "
            << q.w << ", "
            << q.x << ", "
            << q.y << ", "
            << q.z
            << " | Angular Velocity: "
            << angularVelocity.x << ", "
            << angularVelocity.y << ", "
            << angularVelocity.z
            << '\n';
    }

    return 0;
}