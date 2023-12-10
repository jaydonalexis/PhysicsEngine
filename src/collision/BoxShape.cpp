#include <physics/Configuration.h>
#include <physics/collision/BoxShape.h>
#include <physics/collision/AABB.h>

using namespace physics;

/* Constructor */
BoxShape::BoxShape(float hx, float hy, MemoryHandler& memoryHandler) : PolygonShape(memoryHandler) {
  assert(hx > LINEAR_SLOP);
  assert(hy > LINEAR_SLOP);
  set(hx, hy);
}

/* Constructor */
BoxShape::BoxShape(float hx, float hy, const Vector2& center, float angle, MemoryHandler& memoryHandler) : PolygonShape(memoryHandler) {
  assert(hx > LINEAR_SLOP);
  assert(hy > LINEAR_SLOP);
  set(hx, hy, center, angle);
}

/* Get the size of the shape in bytes */
size_t BoxShape::byteSize() const {
  return sizeof(BoxShape);
}

/* Query whether a point is inside the shape */
bool BoxShape::testPoint(const Vector2& pointLocal) const {
  for(uint32 i = 0; i < mNumVertices; i++) {
    if(mNormals[i].dot(pointLocal - mVertices[i]) > 0.0f) {
      return false;
    }
  }

  return true;
}

/* Set the geometric properties of the box */
void BoxShape::set(float hx, float hy) {
  mNumVertices = NUM_VERTICES_BOX;
	mVertices[0].set(-hx, -hy);
	mVertices[1].set( hx, -hy);
	mVertices[2].set( hx,  hy);
	mVertices[3].set(-hx,  hy);
	mNormals[0].set(0.0f, -1.0f);
	mNormals[1].set(1.0f, 0.0f);
	mNormals[2].set(0.0f, 1.0f);
	mNormals[3].set(-1.0f, 0.0f);

  /* Alert broad phase that the geometry of the collision shape has changed */
  alertSizeChange();
}

/* Set the geometric properties of the box */
void BoxShape::set(float hx, float hy, const Vector2& center, float angle) {
  Transform transform;
  mNumVertices = NUM_VERTICES_BOX;
	mVertices[0].set(-hx, -hy);
	mVertices[1].set( hx, -hy);
	mVertices[2].set( hx,  hy);
	mVertices[3].set(-hx,  hy);
	mNormals[0].set(0.0f, -1.0f);
	mNormals[1].set(1.0f, 0.0f);
	mNormals[2].set(0.0f, 1.0f);
	mNormals[3].set(-1.0f, 0.0f);
  transform.setPosition(center);
  transform.setOrientation(angle);

  /* Update the vertices and normals based on the provided position and orientation */
  for(uint32 i = 0; i < mNumVertices; i++) {
    mVertices[i] = transform * mVertices[i];
    mNormals[i] = transform.getOrientation() * mNormals[i];
  }

  /* Alert broad phase that the geometry of the collision shape has changed */
  alertSizeChange();
}

/* Get the rotational inertia of the shape about the local origin */
float BoxShape::getLocalInertia(float mass) const {
  float x = std::fabs(mVertices[0].x);
  float y = std::fabs(mVertices[0].y);
  return (1.0f / 12.0f) * mass * (x * x + y * y);
}

/* Get the area of the shape */
float BoxShape::getArea() const {
  float x = std::fabs(mVertices[0].x);
  float y = std::fabs(mVertices[0].y);
  return x * y;
}

/* Get the centroid of the shape */
Vector2 BoxShape::getCentroid() const {
  return Vector2::getZeroVector();
}