#include "kphysics/math/Quaternion.h"

#include <cmath>

namespace kp {

Quaternion::Quaternion()
    : w(1.0f),
      x(0.0f),
      y(0.0f),
      z(0.0f) {
}

Quaternion::Quaternion(
    float w,
    float x,
    float y,
    float z
)
    : w(w),
      x(x),
      y(y),
      z(z) {
}

Quaternion Quaternion::identity() {
    return Quaternion(
        1.0f,
        0.0f,
        0.0f,
        0.0f
    );
}

Quaternion Quaternion::fromAxisAngle(
    const Vec3& axis,
    float angle
) {
    Vec3 normalizedAxis = axis.normalized();

    float halfAngle = angle * 0.5f;

    float s = std::sin(halfAngle);
    float c = std::cos(halfAngle);

    return Quaternion(
        c,
        normalizedAxis.x * s,
        normalizedAxis.y * s,
        normalizedAxis.z * s
    );
}

Quaternion Quaternion::operator*(
    const Quaternion& q
) const {

    return Quaternion(

        w * q.w -
        x * q.x -
        y * q.y -
        z * q.z,

        w * q.x +
        x * q.w +
        y * q.z -
        z * q.y,

        w * q.y -
        x * q.z +
        y * q.w +
        z * q.x,

        w * q.z +
        x * q.y -
        y * q.x +
        z * q.w
    );
}

Quaternion Quaternion::operator*(
    float scalar
) const {

    return Quaternion(
        w * scalar,
        x * scalar,
        y * scalar,
        z * scalar
    );
}

Quaternion Quaternion::operator+(
    const Quaternion& other
) const {

    return Quaternion(
        w + other.w,
        x + other.x,
        y + other.y,
        z + other.z
    );
}

Quaternion& Quaternion::operator+=(
    const Quaternion& other
) {

    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;

    return *this;
}

void Quaternion::normalize() {

    float length = std::sqrt(
        w * w +
        x * x +
        y * y +
        z * z
    );

    if (length == 0.0f) {
        *this = Quaternion::identity();
        return;
    }

    w /= length;
    x /= length;
    y /= length;
    z /= length;
}

Quaternion Quaternion::normalized() const {

    Quaternion result = *this;

    result.normalize();

    return result;
}

Quaternion Quaternion::conjugate() const {

    return Quaternion(
        w,
        -x,
        -y,
        -z
    );
}

Vec3 Quaternion::rotate(
    const Vec3& vector
) const {

    Quaternion vectorQuaternion(
        0.0f,
        vector.x,
        vector.y,
        vector.z
    );

    Quaternion result =
        (*this) *
        vectorQuaternion *
        conjugate();

    return Vec3(
        result.x,
        result.y,
        result.z
    );
}

Mat3 Quaternion::toMatrix() const {

    float xx = x * x;
    float yy = y * y;
    float zz = z * z;

    float xy = x * y;
    float xz = x * z;
    float yz = y * z;

    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    return Mat3(

        1.0f - 2.0f * (yy + zz),
        2.0f * (xy - wz),
        2.0f * (xz + wy),

        2.0f * (xy + wz),
        1.0f - 2.0f * (xx + zz),
        2.0f * (yz - wx),

        2.0f * (xz - wy),
        2.0f * (yz + wx),
        1.0f - 2.0f * (xx + yy)
    );
}

}