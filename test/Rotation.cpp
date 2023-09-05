#include "UnitTests.h"

#include <physics/mathematics/Rotation.h>
#include <physics/mathematics/Vector2.h>
#include <physics/mathematics/MathCommon.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Rotation, Constructor) {
  Rotation rotation1;
  EXPECT_TRUE(approximateEqual(rotation1.s, 0.0f));
  EXPECT_TRUE(approximateEqual(rotation1.c, 1.0f));

  Rotation rotation2{0.5f};
  EXPECT_TRUE(approximateEqual(rotation2.s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(rotation2.c, cosf(0.5f)));
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

TEST(Rotation, Operators) {
  EXPECT_TRUE(Rotation(0, 1) == Rotation(0, 1));
  EXPECT_TRUE(Rotation(0, 1) != Rotation(1, 0));

  Rotation rotation1;
  Rotation rotation2;
  Rotation rotation3;
  rotation1.set(PI_DIV_4);
  rotation2.set(PI_DIV_4);
  rotation3.set(PI_DIV_2);
  EXPECT_TRUE(rotation1 == rotation2);
  EXPECT_TRUE(rotation1 != rotation3);

  rotation3 = rotation1 * rotation2;
  EXPECT_TRUE(approximateEqual(rotation3.get(), PI_DIV_2));

  rotation1 *= rotation2;
  EXPECT_TRUE(approximateEqual(rotation1.get(), PI_DIV_2));

  Rotation rotation4;
  Vector2 vector1{1, 2};
  Vector2 vector2;
  rotation4.set(PI_DIV_4);
  vector2 = rotation4 * vector1;
  EXPECT_TRUE(vector2 == Vector2(-0.7071068f, 2.1213203f));
}

TEST(Rotation, GetAngle) {
  Rotation rotation1{0.5f};
  Rotation rotation2{0.7071068f, 0.7071068f};
  Rotation rotation3{1.0f, 0.0f};
  float angle1 = rotation1.get();
  float angle2 = rotation2.get();
  float angle3 = rotation3.get();
  EXPECT_TRUE(approximateEqual(angle1, 0.5f));
  EXPECT_TRUE(approximateEqual(angle2, PI_DIV_4));
  EXPECT_TRUE(approximateEqual(angle3, PI_DIV_2));
}

TEST(Rotation, GetAxes) {
  Rotation rotation1{PI_DIV_2};
  Rotation rotation2{PI_DIV_4};
  Vector2 vector1 = rotation1.getX();
  Vector2 vector2 = rotation1.getY();
  EXPECT_TRUE(vector1 == Vector2(0, 1));
  EXPECT_TRUE(vector2 == Vector2(-1, 0));
  
  vector1 = rotation2.getX();
  vector2 = rotation2.getY();
  
  EXPECT_TRUE(vector1 == Vector2(0.7071068f, 0.7071068f));
  EXPECT_TRUE(vector2 == Vector2(-0.7071068f, 0.7071068f));
}

TEST(Rotation, GetZero) {
  Rotation rotation1{PI_DIV_4};
  Rotation rotation2{PI_DIV_4};
  rotation2 = rotation1.getZero();
  EXPECT_TRUE(rotation2 == Rotation(0, 1));
}

TEST(Rotation, TransposeMultiply) {
  Rotation rotation1{PI_DIV_8};
  Rotation rotation2{PI_DIV_4};
  Rotation rotation3;
  rotation3 = transposeMultiply(rotation1, rotation2);
  EXPECT_TRUE(approximateEqual(rotation3.get(), 0.3926990f));

  Rotation rotation4{PI_DIV_8};
  Vector2 vector1{1, 2};
  Vector2 vector2;
  vector2 = transposeMultiply(rotation4, vector1);
  EXPECT_TRUE(vector2 == Vector2(1.68924640f, 1.46507563f));
  std::cout << vector2.x << " " << vector2.y << std::endl;
}
