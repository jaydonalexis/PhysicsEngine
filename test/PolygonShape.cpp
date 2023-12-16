#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(PolygonShape, Constructor) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 1.0f), Vector2(2.0f, 0.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->getType(), ShapeType::Polygon);
}

TEST(PolygonShape, ByteSize) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->byteSize(), sizeof(PolygonShape));
}

TEST(PolygonShape, Radius) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->getRadius(), POLYGON_RADIUS);
}

TEST(PolygonShape, TestPoint) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_TRUE(polygon->testPoint(Vector2(0.2f, 0.2f)));
  EXPECT_FALSE(polygon->testPoint(Vector2(2.0f, 2.0f)));
}

TEST(PolygonShape, NumberOfVertices) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->getNumVertices(), 3);
}

TEST(PolygonShape, VertexPosition) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->getVertexPosition(0), Vector2(0.0f, 0.0f));
  EXPECT_EQ(polygon->getVertexPosition(1), Vector2(1.0f, 0.0f));
  EXPECT_EQ(polygon->getVertexPosition(2), Vector2(0.0f, 1.0f));
}

TEST(PolygonShape, EdgeNormal) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 3);
  EXPECT_EQ(polygon->getEdgeNormal(0), Vector2(0.0f, -1.0f));
  EXPECT_EQ(polygon->getEdgeNormal(1), Vector2((float)std::sqrt(2) / 2, (float)std::sqrt(2) / 2));
  EXPECT_EQ(polygon->getEdgeNormal(2), Vector2(-1.0f, 0.0f));
}

TEST(PolygonShape, Area) {
  Factory factory;
  Vector2 points1[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f), Vector2(0.0f, 1.0f)};
  Vector2 points2[] = {Vector2(0.0f, 0.0f), Vector2(0.0f, 2.0f), Vector2(2.0f, 2.0f), Vector2(2.0f, 0.0f)};
  PolygonShape* polygon1 = factory.createPolygon(points1, 3);
  PolygonShape* polygon2 = factory.createPolygon(points2, 4);
  EXPECT_EQ(polygon1->getArea(), 0.5f);
  EXPECT_EQ(polygon2->getArea(), 4.0f);
}

TEST(PolygonShape, Centroid) {
  Factory factory;
  Vector2 points1[] = {Vector2(0.0f, 0.0f), Vector2(1.0f, 1.0f), Vector2(2.0f, 0.0f)};
  Vector2 points2[] = {Vector2(0.0f, 0.0f), Vector2(0.0f, 1.0f), Vector2(1.0f, 1.0f), Vector2(1.0f, 0.0f)};
  PolygonShape* polygon1 = factory.createPolygon(points1, 3);
  PolygonShape* polygon2 = factory.createPolygon(points2, 4);
  EXPECT_EQ(polygon1->getCentroid(), Vector2(1.0f, (1.0f / 3.0f)));
  EXPECT_EQ(polygon2->getCentroid(), Vector2(0.5f, 0.5f));
}

TEST(PolygonShape, Inertia) {
  Factory factory;
  float mass = 2.0f;
  Vector2 points1[] = {Vector2(0.0f, 0.0f), Vector2(2.0f, 2.0f), Vector2(4.0f, 0.0f)};
  Vector2 points2[] = {Vector2(0.0f, 0.0f), Vector2(0.0f, 2.0f), Vector2(2.0f, 2.0f), Vector2(2.0f, 0.0f)};
  PolygonShape* polygon1 = factory.createPolygon(points1, 3);
  PolygonShape* polygon2 = factory.createPolygon(points2, 4);
  Vector2 centroid1 = polygon1->getCentroid();
  Vector2 centroid2 = polygon2->getCentroid();
  float inertia1 = polygon1->getLocalInertia(mass) - mass * (dot(centroid1, centroid1));
  float inertia2 = polygon2->getLocalInertia(mass) - mass * (dot(centroid2, centroid2));
  /* Izz = 1/18 M(b^2 + c^2) */
  EXPECT_TRUE(approximateEqual(inertia1, ((1.0f / 18.0f) * mass * (square(std::sqrt(8.0f)) + square(std::sqrt(8.0f)))), 0.00001f));
  /* Izz = 1/12 M(a^2 + b^2) */
  EXPECT_TRUE(approximateEqual(inertia2, ((1.0f / 12.0f) * mass * (square(2.0f) + square(2.0f))), 0.00001f));
}

TEST(PolygonShape, LocalBounds) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(2.0f, 5.0f), Vector2(3.5f, 6.0f), Vector2(4.5f, 3.0f), Vector2(3.0f, -1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 5);
  Vector2 lowerBound;
  Vector2 upperBound;
  polygon->getLocalBounds(lowerBound, upperBound);
  EXPECT_EQ(lowerBound, Vector2(0.0f, -1.0f) - Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
  EXPECT_EQ(upperBound, Vector2(4.5f, 6.0f) + Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
}

TEST(PolygonShape, AABB) {
  Factory factory;
  Vector2 points[] = {Vector2(0.0f, 0.0f), Vector2(2.0f, 5.0f), Vector2(3.5f, 6.0f), Vector2(4.5f, 3.0f), Vector2(3.0f, -1.0f)};
  PolygonShape* polygon = factory.createPolygon(points, 5);
  AABB aabb;
  Transform transform;
  polygon->computeAABB(aabb, transform);
  EXPECT_EQ(aabb.getlowerBound(), Vector2(0.0f, -1.0f) - Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
  EXPECT_EQ(aabb.getUpperBound(), Vector2(4.5f, 6.0f) + Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
}