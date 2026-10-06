#include "kphysics/core/RigidBody.h"
#include <cmath>
#include <algorithm>

namespace kp {

RigidBody::RigidBody()
    : position_(0.0f, 0.0f, 0.0f),
      velocity_(0.0f, 0.0f, 0.0f),
      acceleration_(0.0f, 0.0f, 0.0f),
      force_(0.0f, 0.0f, 0.0f),

      mass_(1.0f),
      inverseMass_(1.0f),
    restitution_(0.2f),

      orientation_(Quaternion::identity()),

      angularVelocity_(0.0f, 0.0f, 0.0f),

      torque_(0.0f, 0.0f, 0.0f),

    inertiaTensor_(Mat3::identity()),
    worldInverseInertiaTensor_(Mat3::identity()),
    inverseInertiaTensor_(Mat3::identity()) {}

void RigidBody::setMass(float mass) {

    mass_ = mass;

    if (mass <= 0.0f) {
        inverseMass_ = 0.0f;
    } else {
        inverseMass_ = 1.0f / mass;
    }
}

float RigidBody::getMass() const {
    return mass_;
}

float RigidBody::getInverseMass() const {
    return inverseMass_;
}

const Vec3&
RigidBody::getPosition() const {
    return position_;
}

void RigidBody::updateWorldAABB()
{
    const Mat3 rotation = orientation_.toMatrix();

    const float ex =
        std::fabs(rotation.m[0][0]) * halfExtents_.x +
        std::fabs(rotation.m[0][1]) * halfExtents_.y +
        std::fabs(rotation.m[0][2]) * halfExtents_.z;

    const float ey =
        std::fabs(rotation.m[1][0]) * halfExtents_.x +
        std::fabs(rotation.m[1][1]) * halfExtents_.y +
        std::fabs(rotation.m[1][2]) * halfExtents_.z;

    const float ez =
        std::fabs(rotation.m[2][0]) * halfExtents_.x +
        std::fabs(rotation.m[2][1]) * halfExtents_.y +
        std::fabs(rotation.m[2][2]) * halfExtents_.z;

    Vec3 extent(ex, ey, ez);

    worldAABB_ = AABB(
        position_ - extent,
        position_ + extent
    );
}

void RigidBody::updateWorldInverseInertia() {

    Mat3 rotation =
        orientation_.toMatrix();

    Mat3 rotationTranspose =
        rotation.transpose();

    worldInverseInertiaTensor_ =
        rotation *
        inverseInertiaTensor_ *
        rotationTranspose;
}

void RigidBody::setPosition(const Vec3& position)
{
    position_ = position;
    updateWorldAABB();
}

void RigidBody::setOrientation(
    const Quaternion& orientation
) {
    orientation_ = orientation;
    orientation_.normalize();
    updateWorldInverseInertia();
    updateWorldAABB();
}

    void RigidBody::setRestitution(float restitution)
{
    restitution_ = std::clamp(restitution, 0.0f, 1.0f);
}

    float RigidBody::getRestitution() const
{
    return restitution_;
}

const Quaternion&
RigidBody::getOrientation() const {
    return orientation_;
}

const Vec3&
RigidBody::getVelocity() const {
    return velocity_;
}

const Vec3&
RigidBody::getAngularVelocity() const {
    return angularVelocity_;
}

void RigidBody::applyForce(
    const Vec3& force
) {
    force_ += force;
}

void RigidBody::applyForceAtPoint(
    const Vec3& force,
    const Vec3& point
) {
    force_ += force;

    Vec3 relativePosition =
        point - position_;

    Vec3 generatedTorque =
        Vec3::cross(
            relativePosition,
            force
        );

    torque_ += generatedTorque;
}

void RigidBody::clearForces() {
    force_ = Vec3();
}

void RigidBody::applyTorque(
    const Vec3& torque
) {
    torque_ += torque;
}

void RigidBody::clearTorque() {
    torque_ = Vec3();
}

void RigidBody::setInertiaTensor(
    const Mat3& inertia
) {
    inertiaTensor_ = inertia;
    inverseInertiaTensor_ = inertia.inverse();
    updateWorldInverseInertia();
}

void RigidBody::setVelocity(const Vec3& velocity)
{
    velocity_ = velocity;
}

const Mat3&
RigidBody::getInertiaTensor() const {
    return inertiaTensor_;
}

const Mat3&
RigidBody::getInverseInertiaTensor() const {
    return inverseInertiaTensor_;
}

const Mat3&
RigidBody::getWorldInverseInertiaTensor() const {
    return worldInverseInertiaTensor_;
}

void RigidBody::integrate(float dt) {

    if (inverseMass_ == 0.0f) {
        return;
    }

    acceleration_ =
        force_ * inverseMass_;

    velocity_ +=
        acceleration_ * dt;

    position_ +=
        velocity_ * dt;

    Vec3 angularAcceleration =
        worldInverseInertiaTensor_ * torque_;

    angularVelocity_ +=
        angularAcceleration * dt;

    Quaternion angularVelocityQuaternion(
        0.0f,
        angularVelocity_.x,
        angularVelocity_.y,
        angularVelocity_.z
    );

    Quaternion orientationDerivative =
        angularVelocityQuaternion *
        orientation_;

    orientation_ +=
        orientationDerivative *
        (0.5f * dt);

    orientation_.normalize();
    updateWorldInverseInertia();
    updateWorldAABB();
    clearForces();
    clearTorque();
}

void RigidBody::setHalfExtents(const Vec3& halfExtents)
{
    halfExtents_ = halfExtents;
    updateWorldAABB();
}

const Vec3& RigidBody::getHalfExtents() const
{
    return halfExtents_;
}

const AABB& RigidBody::getWorldAABB() const
{
    return worldAABB_;
}

}