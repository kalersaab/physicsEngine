#include "kphysics/physics/Inertia.h"

namespace kp {

    Mat3 Inertia::box(
        float mass,
        float width,
        float height,
        float depth
    ) {
        const float factor = mass / 12.0f;

        const float ix =
            factor * (
                height * height +
                depth * depth
            );

        const float iy =
            factor * (
                width * width +
                depth * depth
            );

        const float iz =
            factor * (
                width * width +
                height * height
            );

        return Mat3(
            ix, 0.0f, 0.0f,
            0.0f, iy, 0.0f,
            0.0f, 0.0f, iz
        );
    }

}