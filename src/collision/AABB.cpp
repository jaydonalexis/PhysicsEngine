#include <physics/collision/AABB.h>

using namespace physics;

/* Constructor */
AABB::AABB(const Vector2& lowerBound, const Vector2& upperBound) : mLowerBound(lowerBound), mUpperBound(upperBound) {}

/* Get the center of the AABB */
Vector2 AABB::getCenter() const {
  return 0.5f * (mLowerBound + mUpperBound);
}

/* Get the half-extents of the AABB */
Vector2 AABB::getHalfExtents() const {
  return 0.5f * (mUpperBound - mLowerBound);
}

/* Debug */
/* Get the extents of the AABB */
Vector2 AABB::getExtents() const {
  return mUpperBound - mLowerBound;
}

/* Get the perimeter of the AABB */
float AABB::getPerimeter() const {
  float x = mUpperBound.x - mLowerBound.x;
  float y = mUpperBound.y - mLowerBound.y;
  return 2.0f * (x + y);
}

/* Get the area of the AABB */
float AABB::getArea() const {
  float x = mUpperBound.x - mLowerBound.x;
  float y = mUpperBound.y - mLowerBound.y;
  return x * y;
}

/* Get the lower bound of the AABB */
const Vector2& AABB::getlowerBound() const {
  return mLowerBound;
}

/* Set the lower bound of the AABB */
void AABB::setLowerBound(const Vector2& lowerBound) {
  mLowerBound = lowerBound;
}

/* Get the upper bound of the AABB */
const Vector2& AABB::getUpperBound() const {
  return mUpperBound;
}

/* Set the upper bound of the AABB */
void AABB::setUpperBound(const Vector2& upperBound) {
  mUpperBound = upperBound;
}

/* Inflate the sides of the AABB */
void AABB::inflate(float dx, float dy) {
  mUpperBound += Vector2(dx, dy);
  mLowerBound -= Vector2(dx, dy);
}

/* Query whether the current AABB is overlapping with the given AABB */
bool AABB::isOverlapping(const AABB& aabb) const {
  if(mUpperBound.x < aabb.mLowerBound.x || aabb.mUpperBound.x < mLowerBound.x) return false;
  if(mUpperBound.y < aabb.mLowerBound.y || aabb.mUpperBound.y < mLowerBound.y) return false;
  return true;
}

/* Combine an AABB with the current one */
void AABB::combine(const AABB& aabb) {
  mLowerBound = min(mLowerBound, aabb.mLowerBound);
  mUpperBound = max(mUpperBound, aabb.mUpperBound);
}

/* Combine two AABBs into the current AABB */
void AABB::combine(const AABB& firstAABB, const AABB& secondAABB) {
  mLowerBound = min(firstAABB.mLowerBound, secondAABB.mLowerBound);
  mUpperBound = max(firstAABB.mUpperBound, secondAABB.mUpperBound);
}

/* Query whether the current AABB contains the given AABB */
bool AABB::contains(const AABB& aabb) const {
  bool inside = true;
  inside = inside && mLowerBound.x <= aabb.mLowerBound.x;
  inside = inside && mLowerBound.y <= aabb.mLowerBound.y;
  inside = inside && aabb.mUpperBound.x <= mUpperBound.x;
  inside = inside && aabb.mUpperBound.y <= mUpperBound.y;
  return inside;
}

/* Apply a scale factor to the AABB */
void AABB::scale(const Vector2& scale) {
  mLowerBound = mLowerBound * scale;
  mUpperBound = mUpperBound * scale;
}