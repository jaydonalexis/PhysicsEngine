#include <physics/Configuration.h>
#include <physics/collision/PolygonShape.h>
#include <physics/collision/AABB.h>

using namespace physics;

/* Constructor */
PolygonShape::PolygonShape(MemoryHandler& memoryHandler) : Shape(ShapeType::Polygon, POLYGON_RADIUS, memoryHandler) {}

/* Constructor */
PolygonShape::PolygonShape(const Vector2* points, uint32 numPoints, MemoryHandler& memoryHandler) : Shape(ShapeType::Polygon, POLYGON_RADIUS, memoryHandler) {
  assert(numPoints >= MIN_POLYGON_VERTICES);
  set(points, numPoints);
}

/* Constructor */
PolygonShape::PolygonShape(const Hull& hull, MemoryHandler& memoryHandler) : Shape(ShapeType::Polygon, POLYGON_RADIUS, memoryHandler) {
  assert(hull.numPoints >= MIN_POLYGON_VERTICES);
  set(hull);
}

/* Get the size of the shape in bytes */
size_t PolygonShape::byteSize() const {
  return sizeof(PolygonShape);
}

/* Query whether a point is inside the shape */
bool PolygonShape::testPoint(const Vector2& localPoint) const {
  for(uint32 i = 0; i < mNumVertices; i++) {
    if(mNormals[i].dot(localPoint - mVertices[i]) > 0.0f) {
      return false;
    }
  }

  return true;
}

/* Set the geometric properties of the polygon */
void PolygonShape::set(const Vector2* points, uint32 numPoints) {
  Hull hull = computeHull(points, numPoints);
  assert(hull.numPoints >= MIN_POLYGON_VERTICES);
  set(hull);
}

/* Set the geometric properties of the polygon */
void PolygonShape::set(const Hull& hull) {
  assert(hull.numPoints >= MIN_POLYGON_VERTICES);
  mNumVertices = hull.numPoints;

  for(uint32 i = 0; i < mNumVertices; i++) {
    mVertices[i] = hull.points[i];
  }

  for(uint32 i = 0; i < mNumVertices; i++) {
    uint32 j = i + 1 < mNumVertices ? i + 1 : 0;
    Vector2 edge = mVertices[j] - mVertices[i];
    assert(edge.lengthSquare() > FLOAT_EPSILON * FLOAT_EPSILON);
    mNormals[i] = edge.cross(1.0f);
    mNormals[i].normalize();
  }

  computeGeometricProperties();
  alertSizeChange();
}

/* https://www.youtube.com/watch?v=uFDTxRv7O8M */
/* https://en.wikipedia.org/wiki/Moment_of_inertia#Calculating_moment_of_inertia_about_an_axis */
/* https://www.youtube.com/watch?v=lCKxeRiBdjQ&t=496s */
/* https://en.wikipedia.org/wiki/Parallel_axis_theorem */

/* Compute geometric properties of the shape */
void PolygonShape::computeGeometricProperties() {
  assert(mNumVertices >= MIN_POLYGON_VERTICES);
  Vector2 centroid(0.0f, 0.0f);
  float area = 0.0f;
  float inertia = 0.0f;
  Vector2 refPoint = mVertices[0];

  for(uint32 i = 0; i < mNumVertices; i++) {
    Vector2 edge1 = mVertices[i] - refPoint;
    Vector2 edge2 = i + 1 < mNumVertices ? mVertices[i + 1] - refPoint : mVertices[0] - refPoint;
    float jacobian = edge1.cross(edge2);
    float subDivisionArea = 0.5f * jacobian;
    area += subDivisionArea;
    centroid += (1.0f / 3.0f) * subDivisionArea * (edge1 + edge2);
    float edge1x = edge1.x;
    float edge1y = edge1.y;
    float edge2x = edge2.x;
    float edge2y = edge2.y;
    float xIntegral = edge1x * edge1x + edge2x * edge1x + edge2x * edge2x;
    float yIntegral = edge1y * edge1y + edge2y * edge1y + edge2y * edge2y;
    inertia += ((1.0f / 12.0f) * jacobian) * (xIntegral + yIntegral);
  }

  /* Cache properties */
  mArea = area;
  mCentroid = ((1.0f / area) * centroid) + refPoint;
  mInertia = (inertia / area) + (mCentroid.dot(mCentroid) - centroid.dot(centroid)); /* Parallel axis theorem */
}

/* Get the number of vertices of the polygon */
uint32 PolygonShape::getNumVertices() const {
  return mNumVertices;
}

/* Get the position of a given vertex */
Vector2 PolygonShape::getVertexPosition(uint32 vertexIndex) const {
  assert(vertexIndex < mNumVertices);
  return mVertices[vertexIndex];
}

/* Get the normal vector of a given edge */
Vector2 PolygonShape::getEdgeNormal(uint32 edgeIndex) {
  assert(edgeIndex < mNumVertices);
  return mNormals[edgeIndex];
}

/* Get the rotational inertia of the shape about the local origin */
float PolygonShape::getLocalInertia(float mass) const {
  return mass * mInertia;
}

/* Get the area of the shape */
float PolygonShape::getArea() const {
  return mArea;
}

/* Get the centroid of the shape */
Vector2 PolygonShape::getCentroid() const {
  return mCentroid;
}

/* Compute the world space AABB of the shape */
void PolygonShape::computeAABB(AABB& aabb, const Transform& transform) const {
  Vector2 lowerBound;
  Vector2 upperBound;
  upperBound = lowerBound = transform * mVertices[0];

  for(uint32 i = 1; i < mNumVertices; i++) {
    Vector2 vector = transform * mVertices[i];
    lowerBound = min(lowerBound, vector);
    upperBound = max(upperBound, vector);
  }

  Vector2 pseudoExtents(mRadius, mRadius);
  aabb.setLowerBound(lowerBound - pseudoExtents);
  aabb.setUpperBound(upperBound + pseudoExtents);
}