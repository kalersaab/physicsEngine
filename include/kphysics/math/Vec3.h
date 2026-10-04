#pragma once

namespace kp {

    class Vec3 {
    public:
        float x;
        float y;
        float z;

        Vec3();
        Vec3(float x, float y, float z);

        Vec3 operator+(const Vec3& other) const;
        Vec3 operator-(const Vec3& other) const;

        Vec3 operator*(float scalar) const;
        Vec3 operator/(float scalar) const;

        Vec3& operator+=(const Vec3& other);
        Vec3& operator-=(const Vec3& other);

        Vec3& operator*=(float scalar);
        Vec3& operator/=(float scalar);

        float length() const;
        float lengthSquared() const;

        Vec3 normalized() const;

        static float dot(const Vec3& a, const Vec3& b);
        static Vec3 cross(const Vec3& a, const Vec3& b);
    };

}