#include "kphysics/physics/ImpulseSolver.h"

#include <algorithm>
#include <cmath>

#include "kphysics/core/RigidBody.h"

namespace kp {

void ImpulseSolver::solve(
    std::vector<Contact>& contacts,
    float dt
)
{
    if (contacts.empty())
        return;

    constexpr int iterations = 8;

    for (int iteration = 0;
         iteration < iterations;
         ++iteration)
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


void ImpulseSolver::solve(
    std::vector<ContactManifold>& manifolds,
    float dt
)
{
    if (manifolds.empty())
        return;

    constexpr int iterations = 10;

    for (int iteration = 0;
         iteration < iterations;
         ++iteration)
    {
        for (ContactManifold& manifold : manifolds)
        {
            const std::size_t contactCount =
                manifold.getContactCount();

            for (std::size_t i = 0;
                 i < contactCount;
                 ++i)
            {
                solveContact(manifold.getContact(i));
            }
        }
    }

    for (ContactManifold& manifold : manifolds)
    {
        const std::size_t contactCount =
            manifold.getContactCount();

        if (contactCount == 0)
            continue;

        const float correctionScale =
            1.0f / static_cast<float>(contactCount);

        for (std::size_t i = 0;
             i < contactCount;
             ++i)
        {
            positionalCorrection(
                manifold.getContact(i),
                correctionScale
            );
        }
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

    const Vec3 normal = contact.normal.normalized();

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
            normal
        );

    if (velocityAlongNormal > 0.0f)
        return;

    const Vec3 raCrossN =
        Vec3::cross(ra, normal);

    const Vec3 rbCrossN =
        Vec3::cross(rb, normal);

    const Vec3 angularA =
        a->getWorldInverseInertiaTensor() *
        raCrossN;

    const Vec3 angularB =
        b->getWorldInverseInertiaTensor() *
        rbCrossN;

    const float angularTermA =
        Vec3::dot(
            Vec3::cross(angularA, ra),
            normal
        );

    const float angularTermB =
        Vec3::dot(
            Vec3::cross(angularB, rb),
            normal
        );

    const float effectiveMass =
        invMassA +
        invMassB +
        angularTermA +
        angularTermB;

    if (effectiveMass <= 0.0f)
        return;

    constexpr float restitutionThreshold = 0.5f;

    float restitution =
        std::min(
            a->getRestitution(),
            b->getRestitution()
        );

    if (std::fabs(velocityAlongNormal)
        < restitutionThreshold)
    {
        restitution = 0.0f;
    }

    float normalImpulseMagnitude =
        -(1.0f + restitution) *
        velocityAlongNormal;

    normalImpulseMagnitude /=
        effectiveMass;

    if (normalImpulseMagnitude < 0.0f)
        return;

    const Vec3 normalImpulse =
        normal *
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
        normal *
        Vec3::dot(
            newRelativeVelocity,
            normal
        );

    const float tangentLength =
        tangent.length();

    if (tangentLength <= 0.000001f)
        return;

    tangent /= tangentLength;

    const Vec3 raCrossT =
        Vec3::cross(ra, tangent);

    const Vec3 rbCrossT =
        Vec3::cross(rb, tangent);

    const Vec3 tangentAngularA =
        a->getWorldInverseInertiaTensor() *
        raCrossT;

    const Vec3 tangentAngularB =
        b->getWorldInverseInertiaTensor() *
        rbCrossT;

    const float tangentAngularTermA =
        Vec3::dot(
            Vec3::cross(
                tangentAngularA,
                ra
            ),
            tangent
        );

    const float tangentAngularTermB =
        Vec3::dot(
            Vec3::cross(
                tangentAngularB,
                rb
            ),
            tangent
        );

    const float tangentMass =
        invMassA +
        invMassB +
        tangentAngularTermA +
        tangentAngularTermB;

    if (tangentMass <= 0.0f)
        return;

    const float tangentVelocity =
        Vec3::dot(
            newRelativeVelocity,
            tangent
        );

    float frictionImpulseMagnitude =
        -tangentVelocity /
        tangentMass;

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

void ImpulseSolver::solveContactWithBias(Contact& contact, float dt)
{
    RigidBody* a = contact.bodyA;
    RigidBody* b = contact.bodyB;

    if (!a || !b)
        return;

    const float invMassA = a->getInverseMass();
    const float invMassB = b->getInverseMass();

    if (invMassA + invMassB <= 0.0f)
        return;

    const Vec3 normal = contact.normal.normalized();

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
            normal
        );

    const Vec3 raCrossN =
        Vec3::cross(ra, normal);

    const Vec3 rbCrossN =
        Vec3::cross(rb, normal);

    const Vec3 angularA =
        a->getWorldInverseInertiaTensor() *
        raCrossN;

    const Vec3 angularB =
        b->getWorldInverseInertiaTensor() *
        rbCrossN;

    const float angularTermA =
        Vec3::dot(
            Vec3::cross(angularA, ra),
            normal
        );

    const float angularTermB =
        Vec3::dot(
            Vec3::cross(angularB, rb),
            normal
        );

    const float effectiveMass =
        invMassA +
        invMassB +
        angularTermA +
        angularTermB;

    if (effectiveMass <= 0.0f)
        return;

    constexpr float restitutionThreshold = 0.5f;

    float restitution =
        std::min(
            a->getRestitution(),
            b->getRestitution()
        );

    if (std::fabs(velocityAlongNormal)
        < restitutionThreshold)
    {
        restitution = 0.0f;
    }

    // Baumgarte stabilization - add bias to push objects apart
    constexpr float baumgarte = 0.4f;
    constexpr float slop = 0.005f;
    const float bias = (baumgarte / dt) * std::max(0.0f, contact.penetration - slop);

    float normalImpulseMagnitude =
        -(1.0f + restitution) * velocityAlongNormal + bias;

    normalImpulseMagnitude /=
        effectiveMass;

    if (normalImpulseMagnitude < 0.0f)
        return;

    const Vec3 normalImpulse =
        normal *
        normalImpulseMagnitude;

    a->applyImpulse(
        -normalImpulse,
        ra
    );

    b->applyImpulse(
        normalImpulse,
        rb
    );

    // Friction
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
        normal *
        Vec3::dot(
            newRelativeVelocity,
            normal
        );

    const float tangentLength =
        tangent.length();

    if (tangentLength <= 0.000001f)
        return;

    tangent /= tangentLength;

    const Vec3 raCrossT =
        Vec3::cross(ra, tangent);

    const Vec3 rbCrossT =
        Vec3::cross(rb, tangent);

    const Vec3 tangentAngularA =
        a->getWorldInverseInertiaTensor() *
        raCrossT;

    const Vec3 tangentAngularB =
        b->getWorldInverseInertiaTensor() *
        rbCrossT;

    const float tangentAngularTermA =
        Vec3::dot(
            Vec3::cross(
                tangentAngularA,
                ra
            ),
            tangent
        );

    const float tangentAngularTermB =
        Vec3::dot(
            Vec3::cross(
                tangentAngularB,
                rb
            ),
            tangent
        );

    const float tangentMass =
        invMassA +
        invMassB +
        tangentAngularTermA +
        tangentAngularTermB;

    if (tangentMass <= 0.0f)
        return;

    const float tangentVelocity =
        Vec3::dot(
            newRelativeVelocity,
            tangent
        );

    float frictionImpulseMagnitude =
        -tangentVelocity /
        tangentMass;

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
    Contact& contact,
    float scale
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

    constexpr float slop = 0.005f;
    constexpr float percent = 0.8f;
    constexpr float maxCorrection = 0.2f;

    const float penetration =
        std::max(
            contact.penetration - slop,
            0.0f
        );

    if (penetration <= 0.0f)
        return;

    const float correctionMagnitude =
        std::min(
            percent *
            penetration /
            totalInverseMass,
            maxCorrection
        ) * scale;

    const Vec3 correction =
        contact.normal.normalized() *
        correctionMagnitude;

    a->setPosition(
        a->getPosition() -
        correction * invMassA
    );

    b->setPosition(
        b->getPosition() +
        correction * invMassB
    );

    a->updateWorldAABB();
    b->updateWorldAABB();
}

}