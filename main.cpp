#include <iostream>

#include "kphysics/core/RigidBody.h"
#include "kphysics/physics/Inertia.h"

int main() {

    constexpr float mass = 2.0f;

    constexpr float width = 2.0f;
    constexpr float height = 4.0f;
    constexpr float depth = 6.0f;

    kp::RigidBody body;

    body.setMass(mass);

    body.setInertiaTensor(
        kp::Inertia::box(
            mass,
            width,
            height,
            depth
        )
    );

    const kp::Mat3& inertia =
        body.getInertiaTensor();

    std::cout
        << "Box inertia:\n";

    std::cout
        << inertia.m[0][0] << " "
        << inertia.m[0][1] << " "
        << inertia.m[0][2] << '\n';

    std::cout
        << inertia.m[1][0] << " "
        << inertia.m[1][1] << " "
        << inertia.m[1][2] << '\n';

    std::cout
        << inertia.m[2][0] << " "
        << inertia.m[2][1] << " "
        << inertia.m[2][2] << '\n';

    return 0;
}