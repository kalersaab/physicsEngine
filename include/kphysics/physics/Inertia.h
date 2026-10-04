#pragma once

#include "kphysics/math/Mat3.h"

namespace kp {

    class Inertia {
    public:
        static Mat3 box(
            float mass,
            float width,
            float height,
            float depth
        );
    };

}