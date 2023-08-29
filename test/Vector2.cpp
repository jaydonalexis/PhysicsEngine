#include "UnitTests.h"

#include <physics/mathematics/Vector2.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Vector2, Constructor) {
  Vector2 vector{1, 2};
  EXPECT_TRUE(approximateEqual(vector.x, 1.0));
  EXPECT_TRUE(approximateEqual(vector.y, 2.0));
}

TEST(Vector2, CopyConstructor) {
  Vector2 vector1{1, 2};
  Vector2 vector2(vector1);
  EXPECT_TRUE(approximateEqual(vector2.x, 1.0));
  EXPECT_TRUE(approximateEqual(vector2.y, 2.0));
}

TEST(Vector2, SetValues) {
  Vector2 vector;
  vector.set(1, 2);
  EXPECT_TRUE(approximateEqual(vector.x, 1.0));
  EXPECT_TRUE(approximateEqual(vector.y, 2.0));
  vector.setZero();
  EXPECT_TRUE(approximateEqual(vector.x, 0.0));
  EXPECT_TRUE(approximateEqual(vector.y, 0.0));
}

TEST(Vector2, Operators) {
  EXPECT_TRUE(Vector2(1, 2) == Vector2(1, 2));
  EXPECT_TRUE(Vector2(1, 2) != Vector2(2, 1));
  
  Vector2 vector1{1, 2};
  EXPECT_TRUE(vector1[0] == 1);
  EXPECT_TRUE(vector1[1] == 2);

  vector1.set(3, 4);
  vector1 = Vector2(1, 2);
  EXPECT_TRUE(vector1 == Vector2(1, 2));

  EXPECT_TRUE(Vector2(1, 2) + Vector2(3, 4) == Vector2(4, 6));
  EXPECT_TRUE(Vector2(1, 2) - Vector2(3, 4) == Vector2(-2, -2));

  Vector2 vector2{1, 2};
  Vector2 vector3{1, 2};
  vector2 += Vector2(3, 4);
  vector3 -= Vector2(3, 4);
  EXPECT_TRUE(vector2 == Vector2(4, 6));
  EXPECT_TRUE(vector3 == Vector2(-2, -2));

  vector2.set(1, 2);
  vector3.set(3, 6);
  EXPECT_TRUE(Vector2(1, 2) * 2 == Vector2(2, 4));
  EXPECT_TRUE(2 * Vector2(1, 2) == Vector2(2, 4));
  EXPECT_TRUE(Vector2(2, 4) / 2 == Vector2(1, 2));

  vector2 *= 2;
  vector3 /= 3;
  EXPECT_TRUE(vector2 == Vector2(2, 4));
  EXPECT_TRUE(vector3 == Vector2(1, 2));

  Vector2 vector4 = vector2 * vector3;
  EXPECT_TRUE(vector4 == Vector2(2, 8));

  Vector2 vector5 = vector2 / vector3;
  EXPECT_TRUE(approximateEqual(vector5.x, 2.0));
  EXPECT_TRUE(approximateEqual(vector5.y, 2.0));

  // Negative operator
  Vector2 vector6(-1, 2);
  Vector2 vector7 = -vector6;
  EXPECT_TRUE(vector7 == Vector2(1, -2));
}

TEST(Vector2, Length) {
  Vector2 vector1{0, 0};
  EXPECT_TRUE(approximateEqual(vector1.length(), 0.0));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 0.0));
  EXPECT_TRUE(vector1.isZeroVector());
  EXPECT_TRUE(vector1.isFiniteVector());

  vector1.set(3, 4);
  EXPECT_TRUE(approximateEqual(vector1.length(), 5.0));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 25.0));
  EXPECT_TRUE(!vector1.isUnitVector());
  EXPECT_TRUE(!vector1.isZeroVector());
  EXPECT_TRUE(vector1.isFiniteVector());

  vector1.set(1, 0);
  EXPECT_TRUE(approximateEqual(vector1.length(), 1.0));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 1.0));
  EXPECT_TRUE(vector1.isUnitVector());

  vector1.set(0, 1);
  EXPECT_TRUE(approximateEqual(vector1.length(), 1.0));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 1.0));
  EXPECT_TRUE(vector1.isUnitVector());

  vector1.set(1, 2);
  Vector2 vector2 = vector1.getUnitVector();
  EXPECT_TRUE(approximateEqual(vector2.length(), 1.0));
  EXPECT_TRUE(approximateEqual(vector2.lengthSquare(), 1.0));
  EXPECT_TRUE(vector2.isUnitVector());
}

TEST(Vector2, Normalize) {
  Vector2 vector1(1, 0);
  Vector2 vector2(0, 1);
  Vector2 vector3(2, 0);
  Vector2 vector4(0, 2);

  vector1.normalize();
  vector2.normalize();
  vector3.normalize();
  vector4.normalize();

  EXPECT_TRUE(vector1 == Vector2(1, 0));
  EXPECT_TRUE(vector2 == Vector2(0, 1));
  EXPECT_TRUE(vector3 == Vector2(1, 0));
  EXPECT_TRUE(vector4 == Vector2(0, 1));
}

TEST(Vector2, DotProduct) {
  EXPECT_TRUE(Vector2(1, 0).dot(Vector2(0, 1)) == 0);
  EXPECT_TRUE(Vector2(1, 1).dot(Vector2(0, 0)) == 0);
  EXPECT_TRUE(Vector2(1, 2).dot(Vector2(2, 1)) == 4);
  EXPECT_TRUE(Vector2(1, 2).dot(Vector2(-2, -1)) == -4);
  EXPECT_TRUE(Vector2(1, 2).dot(Vector2(-2, 1)) == 0);
  EXPECT_TRUE(Vector2(1, 2).dot(Vector2(3, 4)) == 11);
}