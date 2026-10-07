#include "kphysics/physics/ImpulseSolver.h"

#include <algorithm>
#include <cmath>

#include "kphysics/core/RigidBody.h"
#include "kphysics/math/Vec3.h"

namespace kp {

void ImpulseSolver::solve(
    std::vector<Contact>& contacts,
    float dt)
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

    const float invMassA = a->getInverseMass();
    const float invMassB = b->getInverseMass();

    if (invMassA + invMassB <= 0.0f)
        return;

    const Vec3 ra =
        contact.point - a->getPosition();

    const Vec3 rb =
        contact.point - b->getPosition();

    const Vec3 velocityA =
        a->getVelocity() +
        Vec3::cross(
            a->getAngularVelocity(),
            ra
        );

    const Vec3 velocityB =
        b->getVelocity() +
        Vec3::cross(
            b->getAngularVelocity(),
            rb
        );

    const Vec3 relativeVelocity =
        velocityB - velocityA;


    const float velocityAlongNormal =
        Vec3::dot(
            relativeVelocity,
            contact.normal
        );

    if (velocityAlongNormal > 0.0f)
        return;

    constexpr float restitutionThreshold = 1.0f;

    float restitution = std::min(
        a->getRestitution(),
        b->getRestitution()
    );

    if (std::fabs(velocityAlongNormal) < restitutionThreshold)
    {
        restitution = 0.0f;
    }

    const Vec3 raCrossN =
        Vec3::cross(
            ra,
            contact.normal
        );

    const Vec3 rbCrossN =
        Vec3::cross(
            rb,
            contact.normal
        );

    const Vec3 angularA =
        a->getWorldInverseInertiaTensor()
        * raCrossN;

    const Vec3 angularB =
        b->getWorldInverseInertiaTensor()
        * rbCrossN;

    const float rotationalA =
        Vec3::dot(
            Vec3::cross(angularA, ra),
            contact.normal
        );

    const float rotationalB =
        Vec3::dot(
            Vec3::cross(angularB, rb),
            contact.normal
        );

    const float effectiveMass =
        invMassA +
        invMassB +
        rotationalA +
        rotationalB;

    if (effectiveMass <= 0.0f)
        return;

    const float normalImpulseMagnitude =
        -(1.0f + restitution) *
        velocityAlongNormal /
        effectiveMass;

    const Vec3 normalImpulse =
        contact.normal *
        normalImpulseMagnitude;

    a->applyImpulse(
        -normalImpulse,
        ra
    );

    b->applyImpulse(
        normalImpulse,
        rb
    );

    const Vec3 newVelocityA =
        a->getVelocity() +
        Vec3::cross(
            a->getAngularVelocity(),
            ra
        );

    const Vec3 newVelocityB =
        b->getVelocity() +
        Vec3::cross(
            b->getAngularVelocity(),
            rb
        );

    const Vec3 newRelativeVelocity =
        newVelocityB - newVelocityA;

    Vec3 tangent =
        newRelativeVelocity -
        contact.normal *
        Vec3::dot(
            newRelativeVelocity,
            contact.normal
        );

    const float tangentLengthSquared =
        tangent.lengthSquared();

    if (tangentLengthSquared <= 1e-8f)
        return;

    tangent =
        tangent /
        std::sqrt(tangentLengthSquared);

    const Vec3 raCrossT =
        Vec3::cross(ra, tangent);

    const Vec3 rbCrossT =
        Vec3::cross(rb, tangent);

    const Vec3 angularTangentA =
        a->getWorldInverseInertiaTensor()
        * raCrossT;

    const Vec3 angularTangentB =
        b->getWorldInverseInertiaTensor()
        * rbCrossT;

    const float rotationalTangentA =
        Vec3::dot(
            Vec3::cross(
                angularTangentA,
                ra
            ),
            tangent
        );

    const float rotationalTangentB =
        Vec3::dot(
            Vec3::cross(
                angularTangentB,
                rb
            ),
            tangent
        );

    const float tangentEffectiveMass =
        invMassA +
        invMassB +
        rotationalTangentA +
        rotationalTangentB;

    if (tangentEffectiveMass <= 0.0f)
        return;

    float frictionImpulseMagnitude =
        -Vec3::dot(
            newRelativeVelocity,
            tangent
        ) /
        tangentEffectiveMass;

    const float friction =
        std::sqrt(
            a->getFriction() *
            b->getFriction()
        );

    const float maxFriction =
        normalImpulseMagnitude *
        friction;

    frictionImpulseMagnitude =
        std::clamp(
            frictionImpulseMagnitude,
            -maxFriction,
            maxFriction
        );

    const Vec3 frictionImpulse =
        tangent *
        frictionImpulseMagnitude;

    a->applyImpulse(
        -frictionImpulse,
        ra
    );

    b->applyImpulse(
        frictionImpulse,
        rb
    );
}

void ImpulseSolver::positionalCorrection(
    Contact& contact)
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

    constexpr float percent = 0.8f;
    constexpr float slop = 0.005f;

    const float correctionMagnitude =
    std::max(
        contact.penetration - slop,
        0.0f
    ) *
    percent /
    totalInverseMass;

    const float maxCorrection = 0.2f;

    const float clampedCorrection =
        std::min(
            correctionMagnitude,
            maxCorrection
        );

    const Vec3 correction =
        contact.normal *
        clampedCorrection;

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