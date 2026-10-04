#pragma once

#include "kphysics/math/Vec3.h"

namespace kp {

    class Mat3 {
    public:
        float m[3][3];

        Mat3();

        Mat3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22
        );

        static Mat3 identity();

        Vec3 operator*(const Vec3& vector) const;

        Mat3 operator*(const Mat3& other) const;

        Mat3 operator*(float scalar) const;

        Mat3 transpose() const;

        float determinant() const;

        Mat3 inverse() const;
    };

}