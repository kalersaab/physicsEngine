#pragma once

#include <vector>

#include "kphysics/physics/Contact.h"

namespace kp {

    class ImpulseSolver {
    public:
        void solve(
            std::vector<Contact>& contacts,
            float dt
        );

    private:
        void solveContact(Contact& contact);

        void positionalCorrection(Contact& contact);
    };

}