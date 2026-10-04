#pragma once

#include "kphysics/math/Vec3.h"
#include "kphysics/math/Mat3.h"

namespace kp {

    class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;

        Quaternion();

        Quaternion(
            float w,
            float x,
            float y,
            float z
        );

        static Quaternion identity();

        static Quaternion fromAxisAngle(
            const Vec3& axis,
            float angle
        );

        Quaternion operator*(const Quaternion& other) const;

        Quaternion operator*(float scalar) const;

        Quaternion operator+(const Quaternion& other) const;

        Quaternion& operator+=(const Quaternion& other);

        void normalize();

        Quaternion normalized() const;

        Quaternion conjugate() const;

        Vec3 rotate(const Vec3& vector) const;

        Mat3 toMatrix() const;
    };

}