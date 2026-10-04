#include "kphysics/core/RigidBody.h"

namespace kp {

RigidBody::RigidBody()
    : position_(0.0f, 0.0f, 0.0f),
      velocity_(0.0f, 0.0f, 0.0f),
      acceleration_(0.0f, 0.0f, 0.0f),
      force_(0.0f, 0.0f, 0.0f),

      mass_(1.0f),
      inverseMass_(1.0f),

      orientation_(Quaternion::identity()),

      angularVelocity_(0.0f, 0.0f, 0.0f),

      torque_(0.0f, 0.0f, 0.0f),

      inertiaTensor_(Mat3::identity()),
      inverseInertiaTensor_(Mat3::identity()) {
}

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

void RigidBody::setPosition(
    const Vec3& position
) {
    position_ = position;
}

const Vec3&
RigidBody::getPosition() const {
    return position_;
}

void RigidBody::setOrientation(
    const Quaternion& orientation
) {
    orientation_ = orientation;
    orientation_.normalize();
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

    inverseInertiaTensor_ =
        inertia.inverse();
}

const Mat3&
RigidBody::getInertiaTensor() const {
    return inertiaTensor_;
}

const Mat3&
RigidBody::getInverseInertiaTensor() const {
    return inverseInertiaTensor_;
}

void RigidBody::integrate(float dt) {

    if (inverseMass_ == 0.0f) {
        return;
    }

    // Linear dynamics

    acceleration_ =
        force_ * inverseMass_;

    velocity_ +=
        acceleration_ * dt;

    position_ +=
        velocity_ * dt;


    // Angular dynamics

    Vec3 angularAcceleration =
        inverseInertiaTensor_ * torque_;

    angularVelocity_ +=
        angularAcceleration * dt;


    // Quaternion integration

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


    // Clear accumulated forces

    clearForces();
    clearTorque();
}

}