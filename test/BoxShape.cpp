#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(BoxShape, Constructor) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_EQ(box->getType(), ShapeType::Polygon);
}

TEST(BoxShape, ByteSize) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_EQ(box->byteSize(), sizeof(BoxShape));
}

TEST(BoxShape, Radius) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_EQ(box->getRadius(), POLYGON_RADIUS);
}

TEST(BoxShape, TestPoint) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_TRUE(box->testPoint(Vector2(0.2f, 0.2f)));
  EXPECT_FALSE(box->testPoint(Vector2(2.0f, 2.0f)));
}

TEST(BoxShape, NumberOfVertices) {
  Factory factory;
  BoxShape* box = factory.createBox(2.0f, 2.0f);
  EXPECT_EQ(box->getNumVertices(), 4);
}

TEST(BoxShape, VertexPosition) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_EQ(box->getVertexPosition(0), Vector2(-1.0f, -1.0f));
  EXPECT_EQ(box->getVertexPosition(1), Vector2(1.0f, -1.0f));
  EXPECT_EQ(box->getVertexPosition(2), Vector2(1.0f, 1.0f));
  EXPECT_EQ(box->getVertexPosition(3), Vector2(-1.0f, 1.0f));
}

TEST(BoxShape, EdgeNormal) {
  Factory factory;
  BoxShape* box = factory.createBox(1.0f, 1.0f);
  EXPECT_EQ(box->getEdgeNormal(0), Vector2(0.0f, -1.0f));
  EXPECT_EQ(box->getEdgeNormal(1), Vector2(1.0f, 0.0f));
  EXPECT_EQ(box->getEdgeNormal(2), Vector2(0.0f, 1.0f));
  EXPECT_EQ(box->getEdgeNormal(3), Vector2(-1.0f, 0.0f));
}

TEST(BoxShape, Area) {
  Factory factory;
  BoxShape* box = factory.createBox(4.0f, 4.0f);
  EXPECT_EQ(box->getArea(), 16.0f);
}

TEST(BoxShape, Centroid) {
  Factory factory;
  BoxShape* box = factory.createBox(3.0f, 3.0f);
  EXPECT_EQ(box->getCentroid(), Vector2(0.0f, 0.0f));
}

TEST(BoxShape, Inertia) {
  Factory factory;
  BoxShape* box = factory.createBox(3.0f, 3.0f);
  Vector2 centroid = box->getCentroid();
  float inertia = box->getLocalInertia(2.0f) - dot(centroid, centroid);
  EXPECT_EQ(inertia, (1.0f / 12.0f) * 2.0f * (square(3.0f) + square(3.0f)));
}

TEST(BoxShape, LocalBounds) {
  Factory factory;
  BoxShape* box = factory.createBox(3.0f, 3.0f);
  Vector2 lowerBound;
  Vector2 upperBound;
  box->getLocalBounds(lowerBound, upperBound);
  EXPECT_EQ(lowerBound, Vector2(-3.0f, -3.0f) - Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
  EXPECT_EQ(upperBound, Vector2(3.0f, 3.0f) + Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
}

TEST(BoxShape, AABB) {
  Factory factory;
  BoxShape* box = factory.createBox(3.0f, 3.0f);
  AABB aabb;
  Transform transform;
  box->computeAABB(aabb, transform);
  EXPECT_EQ(aabb.getlowerBound(), Vector2(-3.0f, -3.0f) - Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
  EXPECT_EQ(aabb.getUpperBound(), Vector2(3.0f, 3.0f) + Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
}