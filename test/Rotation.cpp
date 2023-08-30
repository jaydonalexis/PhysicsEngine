#include "UnitTests.h"

#include <physics/mathematics/Rotation.h>
#include <physics/mathematics/Vector2.h>
#include <physics/mathematics/MathCommon.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <iostream>

using namespace physics;

TEST(Rotation, Constructor) {
  Rotation rotation1;
  EXPECT_TRUE(rotation1.s == 0.0f);
  EXPECT_TRUE(rotation1.c == 1.0f);
  Rotation rotation2{0.5f};
  EXPECT_TRUE(rotation2.s == sinf(0.5f));
  EXPECT_TRUE(rotation2.c == cosf(0.5f));
}

TEST(Rotation, CopyConstructor) {
  Rotation rotation1{0.5f};
  Rotation rotation2(rotation1);
  EXPECT_TRUE(approximateEqual(rotation2.s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(rotation2.c, cosf(0.5f)));
}

TEST(Rotation, SetAngle) {
  Rotation rotation;
  rotation.set(0.5f);
  EXPECT_TRUE(approximateEqual(rotation.s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(rotation.c, cosf(0.5f)));
}

TEST(Rotation, SetZero) {
  Rotation rotation;
  rotation.set(0.5f);
  rotation.setZero();
  EXPECT_TRUE(approximateEqual(rotation.s, 0.0f));
  EXPECT_TRUE(approximateEqual(rotation.c, 1.0f));
}

TEST(Rotation, GetAngle) {
  Rotation rotation{0.5f};
  float angle = rotation.get();
  EXPECT_TRUE(approximateEqual(angle, 0.5f));
}

TEST(Rotation, GetAxes) {
  Rotation rotation{PI_DIV_2};
  Vector2 vector1 = rotation.getX();
  Vector2 vector2 = rotation.getY();
  EXPECT_TRUE(vector1 == Vector2(0.0f, 1.0f));
  EXPECT_TRUE(vector2 == Vector2(-1.0f, 0.0f));
}

