#include "UnitTests.h"

#include <physics/Physics.h>
#include <physics/collision/AABB.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(AABB, Constructor)
{
  AABB aabb1;
  EXPECT_EQ(aabb1.getlowerBound(), Vector2(0.0f, 0.0f));
  EXPECT_EQ(aabb1.getUpperBound(), Vector2(0.0f, 0.0f));

  AABB aabb2(Vector2(-3.0f, -5.0f), Vector2(65.0f, -1.0f));
  EXPECT_EQ(aabb2.getlowerBound(), Vector2(-3.0f, -5.0f));
  EXPECT_EQ(aabb2.getUpperBound(), Vector2(65.0f, -1.0f));
}

TEST(AABB, Inflate) {
  AABB aabb1(Vector2(-3.0f, 4.0f), Vector2(-1.0f, 6.0f));
  aabb1.inflate(1, 2);
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, -4));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 2));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, 0));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 8));
}

TEST(AABB, Extents) {
  AABB aabb1(Vector2(-10.0f, -10.0f), Vector2(10.0f, 10.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getExtents().x, 20));
  EXPECT_TRUE(approximateEqual(aabb1.getExtents().y, 20));
  
  AABB aabb2(Vector2(-5.0f, 4.0f), Vector2(-2.0f, 20.0f));
  EXPECT_TRUE(approximateEqual(aabb2.getHalfExtents().x, 1.5f));
  EXPECT_TRUE(approximateEqual(aabb2.getHalfExtents().y, 8.0f));
}

TEST(AABB, Center) {
  AABB aabb1(Vector2(-10.0f, -10.0f), Vector2(10.0f, 10.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getCenter().x, 0.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getCenter().y, 0.0f));

  AABB aabb2(Vector2(-5.0f, 4.0f), Vector2(-2.0f, 20.0f));
  EXPECT_TRUE(approximateEqual(aabb2.getCenter().x, -3.5f));
  EXPECT_TRUE(approximateEqual(aabb2.getCenter().y, 12.0f));
}

TEST(AABB, Get) {
  AABB aabb1(Vector2(-12.0f, 34.0f), Vector2(-3.0f, 56.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, -12.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 34.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, -3.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 56.0f));
}

TEST(AABB, Set) {
  AABB aabb1;
  aabb1.setLowerBound(Vector2(-12.0f, 34.0f));
  aabb1.setUpperBound(Vector2(-3.0f, 56.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, -12.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 34.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, -3.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 56.0f));
}

TEST(AABB, Assignment) {
  AABB aabb1;
  AABB aabb2(Vector2(-12.0f, 34.0f), Vector2(-3.0f, 56.0f));
  aabb1 = aabb2;
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, -12.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 34.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, -3.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 56.0f));
}

TEST(AABB, Area) {
  AABB aabb1(Vector2(-10.0f, -10.0f), Vector2(10.0f, 10.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getArea(), 400.0f));
}

TEST(AABB, Scale) {
  AABB aabb1(Vector2(1.0f, 2.0f), Vector2(5.0f, 6.0f));
  aabb1.scale(Vector2(1.0f, 2.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, 1.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 4.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, 5.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 12.0f));
}

TEST(AABB, Merge) {
  AABB aabb1(Vector2(-45.0f, 7.0f), Vector2(23.0f, 8.0f));
  AABB aabb2(Vector2(-15.0f, 6.0f), Vector2(-5.0f, 9.0f));
  AABB aabb3;
  aabb3.combine(aabb1, aabb2);
  EXPECT_TRUE(approximateEqual(aabb3.getlowerBound().x, -45.0f));
  EXPECT_TRUE(approximateEqual(aabb3.getlowerBound().y, 6.0f));
  EXPECT_TRUE(approximateEqual(aabb3.getUpperBound().x, 23.0f));
  EXPECT_TRUE(approximateEqual(aabb3.getUpperBound().y, 9.0f));

  aabb1.combine(aabb2);
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().x, -45.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getlowerBound().y, 6.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().x, 23.0f));
  EXPECT_TRUE(approximateEqual(aabb1.getUpperBound().y, 9.0f));
}

TEST(AABB, Overlap) {
  AABB aabb1(Vector2(-3.0f, -4.0f), Vector2(1.0f, 0.0f));
  EXPECT_TRUE(aabb1.isOverlapping(aabb1));

  AABB aabb2(Vector2(-2.0f, -3.0f), Vector2(-1.0f, 0.0f));
  AABB aabb3(Vector2(-1.0f, -1.0f), Vector2(1.0f, 2.0f));
  EXPECT_TRUE(aabb2.isOverlapping(aabb3));

  AABB aabb4(Vector2(-20.0f, -3.0f), Vector2(-18.0f, 0.0f));
  AABB aabb5(Vector2(-1.0f, -1.0f), Vector2(1.0f, 2.0f));
  EXPECT_FALSE(aabb4.isOverlapping(aabb5));

  AABB aabb6(Vector2(-2.0f, -3.0f), Vector2(-1.0f, 0.0f));
  AABB aabb7(Vector2(-1.0f, 1.0f), Vector2(1.0f, 2.0f));
  EXPECT_FALSE(aabb6.isOverlapping(aabb7));
}

TEST(AABB, Contains) {
  AABB aabb1(Vector2(-3.0f, -4.0f), Vector2(1.0f, 0.0f));
  AABB aabb2(Vector2(0.0f, 0.0f), Vector2(1.0f, 1.0f));
  AABB aabb3(Vector2(-3.0f, -4.0f), Vector2(1.0f, 0.0f));
  AABB aabb4(Vector2(-2.0f, -3.0f), Vector2(0.0f, -1.0f));
  EXPECT_FALSE(aabb1.contains(aabb2));
  EXPECT_TRUE(aabb1.contains(aabb3));
  EXPECT_TRUE(aabb1.contains(aabb4));
}