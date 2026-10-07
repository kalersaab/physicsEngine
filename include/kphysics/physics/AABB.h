#pragma once

#include "kphysics/math/Vec3.h"

namespace kp {

class AABB {
public:
    AABB();
    AABB(const Vec3& min, const Vec3& max);

    const Vec3& getMin() const;
    const Vec3& getMax() const;

    void setMin(const Vec3& min);
    void setMax(const Vec3& max);

    Vec3 center() const;
    Vec3 size() const;
    Vec3 halfSize() const;

    bool contains(const Vec3& point) const;
    bool intersects(const AABB& other) const;

    void expand(const Vec3& point);
    void expand(const AABB& other);

private:
    Vec3 min_;
    Vec3 max_;
};

}