#include <physics/Configuration.h>
#include <physics/collision/CircleShape.h>
#include <physics/collision/AABB.h>
#include <cassert>

using namespace physics;

/* Debug */
/* Able to shift the positions of the shapes relative to local origin? */

/* Constructor */
CircleShape::CircleShape(float radius, MemoryHandler& memoryHandler) : Shape(ShapeType::Circle, radius, memoryHandler) {
  assert(radius > 0.0f);
}

/* Get the size of the shape in bytes */
size_t CircleShape::byteSize() const {
  return sizeof(CircleShape);
}

/* Query whether a point is inside the shape */
bool CircleShape::testPoint(const Vector2& pointLocal) const {
  return pointLocal.lengthSquare() <= mRadius * mRadius;
}

/* Get radius of the sphere */
float CircleShape::getRadius() const {
  return mRadius;
}

/* Set the radius of the sphere */
void CircleShape::setRadius(float radius) {
  assert(radius > 0.0f);
  mRadius = radius;
  /* Alert broad phase that the geometry of the collision shape has changed */
  alertSizeChange();
}

/* Get the rotational inertia of the shape about the local origin */
float CircleShape::getLocalInertia(float mass) const {
  return (0.5f * mass * mRadius * mRadius);
}

/* Get the area of the shape */
float CircleShape::getArea() const {
  return (PI * mRadius * mRadius);
}

/* Get the centroid of the shape */
Vector2 CircleShape::getCentroid() const {
  return Vector2::getZeroVector();
}

/* Get the local bounds of the shape */
void CircleShape::getLocalBounds(Vector2& lowerBound, Vector2& upperBound) const {
  Vector2 extents(mRadius, mRadius);
  upperBound = extents;
  lowerBound = -extents;
}

/* Compute the world space AABB of the shape */
void CircleShape::computeAABB(AABB& aabb, const Transform& transformWorld) const {
  Vector2 extents(mRadius, mRadius);
  aabb.setLowerBound(transformWorld.getPosition() - extents);
  aabb.setUpperBound(transformWorld.getPosition() + extents);
}