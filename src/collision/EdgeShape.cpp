#include <physics/Configuration.h>
#include <physics/collision/EdgeShape.h>
#include <physics/collision/AABB.h>

using namespace physics;

/* Constructor */
EdgeShape::EdgeShape(const Vector2& vertex0, const Vector2& vertex1, MemoryHandler& memoryHandler) : Shape(ShapeType::Edge, POLYGON_RADIUS, memoryHandler) {
  set(vertex0, vertex1);
}

/* Get the size of the shape in bytes */
size_t EdgeShape::byteSize() const {
  return sizeof(EdgeShape);
}

/* Query whether a test point is inside the shape */
bool EdgeShape::testPoint(const Vector2& localPoint) const {
  NOT_USED(localPoint);
  return false;
}

/* Set the geometric properties of the edge */
void EdgeShape::set(const Vector2& vertex0, const Vector2& vertex1) {
  mVertices.first = vertex0;
  mVertices.second = vertex1;
  alertSizeChange();
}

/* Get the vertices of the edge */
Pair<Vector2, Vector2> EdgeShape::getVertices() const {
  return mVertices;
}

/* Get the rotational inertia of the shape about the local origin */
float EdgeShape::getLocalInertia(float mass) const {
  NOT_USED(mass);
  return 0.0f;
}

/* Get the area of the shape */
float EdgeShape::getArea() const {
  return 0.0f;
}

/* Get the centroid of the shape */
Vector2 EdgeShape::getCentroid() const {
  return 0.5f * (mVertices.first + mVertices.second);
}

void EdgeShape::computeAABB(AABB& aabb, const Transform& transform) const {
  Vector2 vector1 = transform * mVertices.first;
  Vector2 vector2 = transform * mVertices.second;
  Vector2 lowerBound = min(vector1, vector2);
  Vector2 upperBound = max(vector1, vector2);
  Vector2 pseudoExtents(mRadius, mRadius);
  aabb.setLowerBound(lowerBound - pseudoExtents);
  aabb.setUpperBound(upperBound + pseudoExtents);
}