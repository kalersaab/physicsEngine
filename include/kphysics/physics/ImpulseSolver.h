#pragma once

#include <vector>

#include "kphysics/physics/Contact.h"
#include "kphysics/physics/ContactManifold.h"

namespace kp {

    class ImpulseSolver {
    public:
        void solve(std::vector<Contact>& contacts, float dt);
        void solve(std::vector<ContactManifold>& manifolds, float dt);

    private:
        void solveContact(Contact& contact);
        void solveContactWithBias(Contact& contact, float dt);

        void positionalCorrection(
            Contact& contact,
            float scale = 1.0f
        );
    };

}