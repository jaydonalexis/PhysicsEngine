#include "UnitTests.h"

#include <physics/mathematics/Vector3.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Vector3, Constructor) {
  Vector3 vector1;
  EXPECT_TRUE(approximateEqual(vector1.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector1.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector1.z, 0.0f));
  
  Vector3 vector2{1, 2, 3};
  EXPECT_TRUE(approximateEqual(vector2.x, 1.0f));
  EXPECT_TRUE(approximateEqual(vector2.y, 2.0f));
  EXPECT_TRUE(approximateEqual(vector2.z, 3.0f));
}

TEST(Vector3, CopyConstructor) {
  Vector3 vector1{1, 2, 3};
  Vector3 vector2(vector1);
  EXPECT_TRUE(approximateEqual(vector2.x, 1.0f));
  EXPECT_TRUE(approximateEqual(vector2.y, 2.0f));
  EXPECT_TRUE(approximateEqual(vector2.z, 3.0f));
}

TEST(Vector3, SetValues) {
  Vector3 vector;
  vector.set(1, 2, 3);
  EXPECT_TRUE(approximateEqual(vector.x, 1.0f));
  EXPECT_TRUE(approximateEqual(vector.y, 2.0f));
  EXPECT_TRUE(approximateEqual(vector.z, 3.0f));

  vector.setZero();
  EXPECT_TRUE(approximateEqual(vector.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector.z, 0.0f));
}

TEST(Vector3, Operators) {
  EXPECT_TRUE(Vector3(1, 2, 3) == Vector3(1, 2, 3));
  EXPECT_TRUE(Vector3(1, 2, 3) != Vector3(2, 1, 3));

  Vector3 vector1{1, 2, 3};
  EXPECT_TRUE(vector1[0] == 1);
  EXPECT_TRUE(vector1[1] == 2);
  EXPECT_TRUE(vector1[2] == 3);

  vector1.set(4, 5, 6);
  vector1 = Vector3(1, 2, 3);
  EXPECT_TRUE(vector1 == Vector3(1, 2, 3));

  EXPECT_TRUE(Vector3(1, 2, 3) + Vector3(4, 5, 6) == Vector3(5, 7, 9));
  EXPECT_TRUE(Vector3(4, 5, 6) - Vector3(1, 2, 3) == Vector3(3, 3, 3));

  Vector3 vector2{1, 2, 3};
  Vector3 vector3{4, 5, 6};
  vector2 += Vector3(4, 5, 6);
  vector3 -= Vector3(1, 2, 3);
  EXPECT_TRUE(vector2 == Vector3(5, 7, 9));
  EXPECT_TRUE(vector3 == Vector3(3, 3, 3));

  EXPECT_TRUE(Vector3(1, 2, 3) * 2 == Vector3(2, 4, 6));
  EXPECT_TRUE(2 * Vector3(1, 2, 3) == Vector3(2, 4, 6));
  EXPECT_TRUE(Vector3(2, 4, 6) / 2 == Vector3(1, 2, 3));

  vector2.set(1, 2, 3);
  vector3.set(3, 6, 9);
  vector2 *= 2;
  vector3 /= 3;
  EXPECT_TRUE(vector2 == Vector3(2, 4, 6));
  EXPECT_TRUE(vector3 == Vector3(1, 2, 3));

  Vector3 vector4 = vector2 * vector3;
  EXPECT_TRUE(vector4 == Vector3(2, 8, 18));

  Vector3 vector5 = vector2 / vector3;
  EXPECT_TRUE(approximateEqual(vector5.x, 2.0f));
  EXPECT_TRUE(approximateEqual(vector5.y, 2.0f));
  EXPECT_TRUE(approximateEqual(vector5.z, 2.0f));

  Vector3 vector6(-1, 2, -3);
  Vector3 vector7 = -vector6;
  EXPECT_TRUE(vector7 == Vector3(1, -2, 3));
}

TEST(Vector3, Length) {
  Vector3 vector1{0, 0, 0};
  EXPECT_TRUE(approximateEqual(vector1.length(), 0.0f));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 0.0f));
  EXPECT_TRUE(vector1.isZeroVector());
  EXPECT_TRUE(vector1.isFiniteVector());

  vector1.set(3, 4, 12);
  EXPECT_TRUE(approximateEqual(vector1.length(), 13.0f));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 169.0f));
  EXPECT_TRUE(!vector1.isUnitVector());
  EXPECT_TRUE(!vector1.isZeroVector());
  EXPECT_TRUE(vector1.isFiniteVector());

  vector1.set(1, 0, 0);
  EXPECT_TRUE(approximateEqual(vector1.length(), 1.0f));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 1.0f));
  EXPECT_TRUE(vector1.isUnitVector());

  vector1.set(0, 1, 0);
  EXPECT_TRUE(approximateEqual(vector1.length(), 1.0f));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 1.0f));
  EXPECT_TRUE(vector1.isUnitVector());

  vector1.set(0, 0, 1);
  EXPECT_TRUE(approximateEqual(vector1.length(), 1.0f));
  EXPECT_TRUE(approximateEqual(vector1.lengthSquare(), 1.0f));
  EXPECT_TRUE(vector1.isUnitVector());

  vector1.set(1, 2, 3);
  Vector3 vector2 = vector1.getUnitVector();
  EXPECT_TRUE(approximateEqual(vector2.length(), 1.0f));
}

TEST(Vector3, Normalize) {
  Vector3 vector1(1, 0, 0);
  Vector3 vector2(0, 1, 0);
  Vector3 vector3(0, 0, 1);
  Vector3 vector4(2, 0, 0);
  Vector3 vector5(0, 2, 0);
  Vector3 vector6(0, 0, 2);

  vector1.normalize();
  vector2.normalize();
  vector3.normalize();
  vector4.normalize();
  vector5.normalize();
  vector6.normalize();

  EXPECT_TRUE(approximateEqual(vector1.x, 1.0f));
  EXPECT_TRUE(approximateEqual(vector1.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector1.z, 0.0f));

  EXPECT_TRUE(approximateEqual(vector2.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector2.y, 1.0f));
  EXPECT_TRUE(approximateEqual(vector2.z, 0.0f));

  EXPECT_TRUE(approximateEqual(vector3.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector3.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector3.z, 1.0f));

  EXPECT_TRUE(approximateEqual(vector4.x, 1.0f));
  EXPECT_TRUE(approximateEqual(vector4.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector4.z, 0.0f));

  EXPECT_TRUE(approximateEqual(vector5.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector5.y, 1.0f));
  EXPECT_TRUE(approximateEqual(vector5.z, 0.0f));

  EXPECT_TRUE(approximateEqual(vector6.x, 0.0f));
  EXPECT_TRUE(approximateEqual(vector6.y, 0.0f));
  EXPECT_TRUE(approximateEqual(vector6.z, 1.0f));
}

TEST(Vector3, DotProduct) {
  EXPECT_TRUE(Vector3(1, 0, 0).dot(Vector3(0, 1, 0)) == 0);
  EXPECT_TRUE(Vector3(1, 1, 0).dot(Vector3(0, 0, 1)) == 0);
  EXPECT_TRUE(Vector3(1, 2, 3).dot(Vector3(2, 1, 3)) == 13);
  EXPECT_TRUE(Vector3(1, 2, 3).dot(Vector3(-2, -1, -3)) == -13);
  EXPECT_TRUE(Vector3(1, 2, 3).dot(Vector3(-2, 1, 3)) == 9);
  EXPECT_TRUE(Vector3(1, 2, 3).dot(Vector3(3, 4, 5)) == 26);
}