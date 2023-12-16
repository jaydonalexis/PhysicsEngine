#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(CircleShape, Constructor) {
  Factory factory;
  CircleShape* circle = factory.createCircle(1.0f);
  EXPECT_EQ(circle->getType(), ShapeType::Circle);
}

TEST(CircleShape, ByteSize) {
  Factory factory;
  CircleShape* circle = factory.createCircle(1.0f);
  EXPECT_EQ(circle->byteSize(), sizeof(CircleShape));
}

TEST(CircleShape, Radius) {
  Factory factory;
  CircleShape* circle = factory.createCircle(3.0f);
  EXPECT_EQ(circle->getRadius(), 3.0f);
}

TEST(CircleShape, TestPoint) {
  Factory factory;
  CircleShape* circle = factory.createCircle(3.0f);
  EXPECT_TRUE(circle->testPoint(Vector2(-2.9f, 0.0f)));
  EXPECT_FALSE(circle->testPoint(Vector2(0.0f, 3.1f)));
}

TEST(CircleShape, Area) {
  Factory factory;
  CircleShape* circle = factory.createCircle(2.5f);
  EXPECT_EQ(circle->getArea(), PI * square(2.5f));
}

TEST(CircleShape, Centroid) {
  Factory factory;
  CircleShape* circle = factory.createCircle(4.0f);
  EXPECT_EQ(circle->getCentroid(), Vector2(0.0f, 0.0f));
}

TEST(CircleShape, Inertia) {
  Factory factory;
  CircleShape* circle = factory.createCircle(4.0f);
  Vector2 centroid = circle->getCentroid();
  float inertia = circle->getLocalInertia(2.0f) - dot(centroid, centroid);
  EXPECT_EQ(inertia, 0.5f * 2.0f * square(4.0f));
}

TEST(CircleShape, LocalBounds) {
  Factory factory;
  CircleShape* circle = factory.createCircle(5.0f);
  Vector2 lowerBound;
  Vector2 upperBound;
  circle->getLocalBounds(lowerBound, upperBound);
  EXPECT_EQ(lowerBound, Vector2(-5.0f, -5.0f));
  EXPECT_EQ(upperBound, Vector2(5.0f, 5.0f));
}

TEST(CircleShape, AABB) {
  Factory factory;
  CircleShape* circle = factory.createCircle(5.0f);
  AABB aabb;
  Transform transform;
  circle->computeAABB(aabb, transform);
  EXPECT_EQ(aabb.getlowerBound(), Vector2(-5.0f, -5.0f));
  EXPECT_EQ(aabb.getUpperBound(), Vector2(5.0f, 5.0f));
}