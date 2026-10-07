#include "kphysics/physics/AABB.h"

namespace kp {

AABB::AABB()
    : min_(0.0f, 0.0f, 0.0f),
      max_(0.0f, 0.0f, 0.0f)
{
}

AABB::AABB(const Vec3& min, const Vec3& max)
    : min_(min),
      max_(max)
{
}

const Vec3& AABB::getMin() const
{
    return min_;
}

const Vec3& AABB::getMax() const
{
    return max_;
}

void AABB::setMin(const Vec3& min)
{
    min_ = min;
}

void AABB::setMax(const Vec3& max)
{
    max_ = max;
}

Vec3 AABB::center() const
{
    return (min_ + max_) * 0.5f;
}

Vec3 AABB::size() const
{
    return max_ - min_;
}

Vec3 AABB::halfSize() const
{
    return size() * 0.5f;
}

bool AABB::contains(const Vec3& point) const
{
    return
        point.x >= min_.x &&
        point.x <= max_.x &&

        point.y >= min_.y &&
        point.y <= max_.y &&

        point.z >= min_.z &&
        point.z <= max_.z;
}

bool AABB::intersects(const AABB& other) const
{
    return
        min_.x <= other.max_.x &&
        max_.x >= other.min_.x &&

        min_.y <= other.max_.y &&
        max_.y >= other.min_.y &&

        min_.z <= other.max_.z &&
        max_.z >= other.min_.z;
}

void AABB::expand(const Vec3& point)
{
    if (point.x < min_.x)
        min_.x = point.x;

    if (point.y < min_.y)
        min_.y = point.y;

    if (point.z < min_.z)
        min_.z = point.z;

    if (point.x > max_.x)
        max_.x = point.x;

    if (point.y > max_.y)
        max_.y = point.y;

    if (point.z > max_.z)
        max_.z = point.z;
}

void AABB::expand(const AABB& other)
{
    expand(other.min_);
    expand(other.max_);
}

}