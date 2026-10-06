#include "kphysics/physics/ImpulseSolver.h"
#include "kphysics/core/RigidBody.h"

#include <algorithm>
#include <cmath>

namespace kp {

void ImpulseSolver::solve(
    std::vector<Contact>& contacts,
    float dt
)
{
    constexpr int iterations = 8;

    for (int i = 0; i < iterations; ++i)
    {
        for (Contact& contact : contacts)
        {
            solveContact(contact);
        }
    }

    for (Contact& contact : contacts)
    {
        positionalCorrection(contact);
    }

    (void)dt;
}

void ImpulseSolver::solveContact(Contact& contact)
{
    RigidBody* a = contact.bodyA;
    RigidBody* b = contact.bodyB;

    if (!a || !b)
        return;

    const float invMassA =
        a->getInverseMass();

    const float invMassB =
        b->getInverseMass();

    const float totalInverseMass =
        invMassA + invMassB;

    if (totalInverseMass <= 0.0f)
        return;

    const Vec3 relativeVelocity =
        b->getVelocity() -
        a->getVelocity();

    const float velocityAlongNormal =
        Vec3::dot(
            relativeVelocity,
            contact.normal
        );

    if (velocityAlongNormal > 0.0f)
        return;

    const float restitution =
        std::min(
            a->getRestitution(),
            b->getRestitution()
        );

    const float impulseMagnitude =
        -(1.0f + restitution) *
        velocityAlongNormal /
        totalInverseMass;

    const Vec3 impulse =
        contact.normal *
        impulseMagnitude;

    a->setVelocity(
        a->getVelocity() -
        impulse * invMassA
    );

    b->setVelocity(
        b->getVelocity() +
        impulse * invMassB
    );
}

void ImpulseSolver::positionalCorrection(
    Contact& contact
)
{
    RigidBody* a = contact.bodyA;
    RigidBody* b = contact.bodyB;

    if (!a || !b)
        return;

    const float invMassA =
        a->getInverseMass();

    const float invMassB =
        b->getInverseMass();

    const float totalInverseMass =
        invMassA + invMassB;

    if (totalInverseMass <= 0.0f)
        return;

    constexpr float percent = 1.0f;
    constexpr float slop = 0.001f;

    const float correctionMagnitude =
        std::max(
            contact.penetration - slop,
            0.0f
        ) *
        percent /
        totalInverseMass;

    const Vec3 correction =
        contact.normal *
        correctionMagnitude;

    a->setPosition(
        a->getPosition() -
        correction * invMassA
    );

    b->setPosition(
        b->getPosition() +
        correction * invMassB
    );
}

}