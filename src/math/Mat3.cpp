#include "kphysics/math/Mat3.h"

#include <stdexcept>

namespace kp {

Mat3::Mat3()
    : m{
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    } {
}

Mat3::Mat3(
    float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22
) : m{
        {m00, m01, m02},
        {m10, m11, m12},
        {m20, m21, m22}
    } {
}

Mat3 Mat3::identity() {

    return Mat3(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    );
}

Vec3 Mat3::operator*(const Vec3& vector) const {

    return Vec3(
        m[0][0] * vector.x +
        m[0][1] * vector.y +
        m[0][2] * vector.z,

        m[1][0] * vector.x +
        m[1][1] * vector.y +
        m[1][2] * vector.z,

        m[2][0] * vector.x +
        m[2][1] * vector.y +
        m[2][2] * vector.z
    );
}

Mat3 Mat3::operator*(const Mat3& other) const {

    Mat3 result;

    for (int row = 0; row < 3; ++row) {

        for (int column = 0; column < 3; ++column) {

            result.m[row][column] =
                m[row][0] * other.m[0][column] +
                m[row][1] * other.m[1][column] +
                m[row][2] * other.m[2][column];
        }
    }

    return result;
}

Mat3 Mat3::operator*(float scalar) const {

    Mat3 result;

    for (int row = 0; row < 3; ++row) {

        for (int column = 0; column < 3; ++column) {

            result.m[row][column] =
                m[row][column] * scalar;
        }
    }

    return result;
}

Mat3 Mat3::transpose() const {

    return Mat3(
        m[0][0], m[1][0], m[2][0],
        m[0][1], m[1][1], m[2][1],
        m[0][2], m[1][2], m[2][2]
    );
}

float Mat3::determinant() const {

    return
        m[0][0] * (
            m[1][1] * m[2][2] -
            m[1][2] * m[2][1]
        )
        -
        m[0][1] * (
            m[1][0] * m[2][2] -
            m[1][2] * m[2][0]
        )
        +
        m[0][2] * (
            m[1][0] * m[2][1] -
            m[1][1] * m[2][0]
        );
}

Mat3 Mat3::inverse() const {

    float det = determinant();

    if (det == 0.0f) {
        throw std::runtime_error(
            "Mat3: matrix is not invertible"
        );
    }

    float invDet = 1.0f / det;

    return Mat3(

        (m[1][1] * m[2][2] -
         m[1][2] * m[2][1]) * invDet,

        (m[0][2] * m[2][1] -
         m[0][1] * m[2][2]) * invDet,

        (m[0][1] * m[1][2] -
         m[0][2] * m[1][1]) * invDet,

        (m[1][2] * m[2][0] -
         m[1][0] * m[2][2]) * invDet,

        (m[0][0] * m[2][2] -
         m[0][2] * m[2][0]) * invDet,

        (m[0][2] * m[1][0] -
         m[0][0] * m[1][2]) * invDet,

        (m[1][0] * m[2][1] -
         m[1][1] * m[2][0]) * invDet,

        (m[0][1] * m[2][0] -
         m[0][0] * m[2][1]) * invDet,

        (m[0][0] * m[1][1] -
         m[0][1] * m[1][0]) * invDet
    );
}

}