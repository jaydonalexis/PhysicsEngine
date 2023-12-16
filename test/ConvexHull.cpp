#include "UnitTests.h"

#include <physics/mathematics/ConvexHull.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(Hull, AllPointsInHull) {
  Vector2 points[] = {Vector2(0.0f, -3.0f), Vector2(-1.0f, 1.0f), Vector2(2.0f, 4.0f), Vector2(8.0f, 2.0f), Vector2(5.0f, -1.0f)};
  Hull hull = getHull(points, 5);
  EXPECT_EQ(hull.numPoints, 5);
}

TEST(Hull, SomePointsInsideHull) {
  Vector2 points[] = {Vector2(0.0f, -3.0f), Vector2(-1.0f, 1.0f), Vector2(2.0f, 4.0f), Vector2(8.0f, 2.0f), Vector2(5.0f, -1.0f), Vector2(2.0f, 2.0f)};
  Hull hull = getHull(points, 6);
  EXPECT_EQ(hull.numPoints, 5);
}

TEST(Hull, CollinearPoints) {
  Vector2 points[] = {Vector2(1.0f, 1.0f), Vector2(2.0f, 2.0f), Vector2(3.0f, 3.0f)};
  Hull hull = getHull(points, 3);
  EXPECT_EQ(hull.numPoints, 0);
}

TEST(Hull, VeryClosePoints) {
  Vector2 points[] = {Vector2(-1.0f, 1.0f), Vector2(-1.0001f, 1.0001f), Vector2(-1.0002f, 1.0002f)};
  Hull hull = getHull(points, 3);
  EXPECT_EQ(hull.numPoints, 0);
}

TEST(Hull, OnePointProvided) {
  Vector2 points[] = {Vector2(0.0f, 0.0f)};
  Hull hull = getHull(points, 1);
  EXPECT_EQ(hull.numPoints, 0);
}