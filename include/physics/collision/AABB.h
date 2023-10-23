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
    void combine(const AABB& aabb1, const AABB& aabb2);

    /* Query whether the current AABB contains the given AABB */
    bool contains(const AABB& aabb) const;

    /* Apply a scale factor to the AABB */
    void scale(const Vector2& scale);

    /* -- Friends -- */

    friend class DynamicTree;
};

/* Constructor */
inline AABB::AABB(const Vector2& lowerBound, const Vector2& upperBound) : mLowerBound(lowerBound), mUpperBound(upperBound) {}

/* Get the center of the AABB */
inline Vector2 AABB::getCenter() const {
  return 0.5f * (mLowerBound + mUpperBound);
}

/* Get the half-extents of the AABB */
inline Vector2 AABB::getHalfExtents() const {
  return 0.5f * (mUpperBound - mLowerBound);
}

/* Get the extents of the AABB */
inline Vector2 AABB::getExtents() const {
  return mUpperBound - mLowerBound;
}

/* Get the perimeter of the AABB */
inline float AABB::getPerimeter() const {
  float x = mUpperBound.x - mLowerBound.x;
  float y = mUpperBound.y - mLowerBound.y;
  return 2.0f * (x + y);
}

/* Get the area of the AABB */
inline float AABB::getArea() const {
  float x = mUpperBound.x - mLowerBound.x;
  float y = mUpperBound.y - mLowerBound.y;
  return x * y;
}

/* Get the lower bound of the AABB */
inline const Vector2& AABB::getlowerBound() const {
  return mLowerBound;
}

/* Set the lower bound of the AABB */
inline void AABB::setLowerBound(const Vector2& lowerBound) {
  mLowerBound = lowerBound;
}

/* Get the upper bound of the AABB */
inline const Vector2& AABB::getUpperBound() const {
  return mUpperBound;
}

/* Set the upper bound of the AABB */
inline void AABB::setUpperBound(const Vector2& upperBound) {
  mUpperBound = upperBound;
}

/* Inflate the sides of the AABB */
inline void AABB::inflate(float dx, float dy) {
  mUpperBound += Vector2(dx, dy);
  mLowerBound -= Vector2(dx, dy);
}

/* Query whether the current AABB is overlapping with the given AABB */
inline bool AABB::isOverlapping(const AABB& aabb) const {
  if(mUpperBound.x < aabb.mLowerBound.x || aabb.mUpperBound.x < mLowerBound.x) return false;
  if(mUpperBound.y < aabb.mLowerBound.y || aabb.mUpperBound.y < mLowerBound.y) return false;
  return true;
}

/* Combine an AABB with the current one */
inline void AABB::combine(const AABB& aabb) {
  mLowerBound.x = std::min(mLowerBound.x, aabb.mLowerBound.x);
  mLowerBound.y = std::min(mLowerBound.y, aabb.mLowerBound.y);
  mUpperBound.x = std::max(mUpperBound.x, aabb.mUpperBound.x);
  mUpperBound.y = std::max(mUpperBound.y, aabb.mUpperBound.y);
}

/* Combine two AABBs into the current AABB */
inline void AABB::combine(const AABB& aabb1, const AABB& aabb2) {
  mLowerBound.x = std::min(aabb1.mLowerBound.x, aabb2.mLowerBound.x);
  mLowerBound.y = std::min(aabb1.mLowerBound.y, aabb2.mLowerBound.y);
  mUpperBound.x = std::max(aabb1.mUpperBound.x, aabb2.mUpperBound.x);
  mUpperBound.y = std::max(aabb1.mUpperBound.y, aabb2.mUpperBound.y);
}

/* Query whether the current AABB contains the given AABB */
inline bool AABB::contains(const AABB& aabb) const {
  bool inside = true;
  inside = inside && mLowerBound.x <= aabb.mLowerBound.x;
  inside = inside && mLowerBound.y <= aabb.mLowerBound.y;
  inside = inside && mUpperBound.x >= aabb.mUpperBound.x;
  inside = inside && mUpperBound.y >= aabb.mUpperBound.y;
  return inside;
}

/* Apply a scale factor to the AABB */
inline void AABB::scale(const Vector2& scale) {
  mLowerBound = mLowerBound * scale;
  mUpperBound = mUpperBound * scale;
}

}

#endif