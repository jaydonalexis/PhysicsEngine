#ifndef PHYSICS_AABB_H
#define PHYSICS_AABB_H

#include <physics/mathematics/Math.h>

namespace physics {

class AABB {

  private:
    /* -- Attributes -- */

    /* Lower bound */
    Vector2 mLowerBound;

    /* Upper bound */
    Vector2 mUpperBound;

  public:
    /* -- Methods -- */

    /* Constructor */
    AABB() = default;

    /* Constructor */
    AABB(const Vector2& lowerBound, const Vector2& upperBound);

    /* Get the center of the AABB */
    Vector2 getCenter() const;

    /* Get the half-extents of the AABB */
    Vector2 getHalfExtents() const;

    /* Get the extents of the AABB */
    Vector2 getExtents() const;

    /* Get the perimeter of the AABB */
    float getPerimeter() const;

    /* Get the area of the AABB */
    float getArea() const;

    /* Get the lower bound of the AABB */
    const Vector2& getlowerBound() const;

    /* Set the lower bound of the AABB */
    void setLowerBound(const Vector2& lowerBound);
    
    /* Get the upper bound of the AABB */
    const Vector2& getUpperBound() const;

    /* Set the upper bound of the AABB */
    void setUpperBound(const Vector2& upperBound);

    /* Inflate the sides of the AABB */
    void inflate(float dx, float dy);

    /* Query whether the current AABB is overlapping with the given AABB */
    bool isOverlapping(const AABB& aabb) const;

    /* Combine an AABB with the current one */
    void combine(const AABB& aabb);

    /* Combine two AABBs into the current AABB */
    void combine(const AABB& firstAABB, const AABB& secondAABB);

    /* Query whether the current AABB contains the given AABB */
    bool contains(const AABB& aabb) const;

    /* Apply a scale factor to the AABB */
    void scale(const Vector2& scale);

    /* -- Friends -- */

    friend class DynamicTree;
};

}

#endif