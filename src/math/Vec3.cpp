#include "kphysics/math/Vec3.h"

#include <cmath>

namespace kp {

    Vec3::Vec3()
        : x(0.0f), y(0.0f), z(0.0f) {}

    Vec3::Vec3(float x, float y, float z)
        : x(x), y(y), z(z) {}

    Vec3 Vec3::operator+(const Vec3& other) const {
        return {
            x + other.x,
            y + other.y,
            z + other.z
        };
    }

    Vec3 Vec3::operator-(const Vec3& other) const {
        return {
            x - other.x,
            y - other.y,
            z - other.z
        };
    }

    Vec3 Vec3::operator*(float scalar) const {
        return {
            x * scalar,
            y * scalar,
            z * scalar
        };
    }

    Vec3 Vec3::operator/(float scalar) const {
        return {
            x / scalar,
            y / scalar,
            z / scalar
        };
    }

    Vec3& Vec3::operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }

    Vec3& Vec3::operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;

        return *this;
    }

    Vec3& Vec3::operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;

        return *this;
    }

    float Vec3::lengthSquared() const {
        return x * x + y * y + z * z;
    }

    float Vec3::length() const {
        return std::sqrt(lengthSquared());
    }

    Vec3 Vec3::normalized() const {
        float len = length();

        if (len == 0.0f) {
            return Vec3();
        }

        return *this / len;
    }

    float Vec3::dot(const Vec3& a, const Vec3& b) {
        return
            a.x * b.x +
            a.y * b.y +
            a.z * b.z;
    }

    Vec3 Vec3::cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

}