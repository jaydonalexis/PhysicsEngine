#include "UnitTests.h"

#include <physics/mathematics/Matrix22.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Matrix22, Constructor) {
  Matrix22 matrix1;
  EXPECT_TRUE(approximateEqual(matrix1[0][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[1][1], 0.0f));

  Matrix22 matrix2{1};
  EXPECT_TRUE(approximateEqual(matrix2[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][1], 1.0f));

  Matrix22 matrix3{1, 2, 3, 4};
  EXPECT_TRUE(approximateEqual(matrix3[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix3[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix3[1][0], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix3[1][1], 4.0f));
}

TEST(Matrix22, CopyConstructor) {
  Matrix22 matrix1{1, 2, 3, 4};
  Matrix22 matrix2(matrix1);
  EXPECT_TRUE(approximateEqual(matrix2[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][0], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][1], 4.0f));
}

TEST(Matrix22, SetValues) {
  Matrix22 matrix;
  matrix.set(1);
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 1.0f));

  matrix.set(1, 2, 3, 4);
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 4.0f));

  matrix.setZero();
  EXPECT_TRUE(approximateEqual(matrix[0][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 0.0f));

  matrix.setIdentity();
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 1.0f));
}

TEST(Matrix22, Operators) {
  EXPECT_TRUE(Matrix22(1, 2, 3, 4) == Matrix22(1, 2, 3, 4));
  EXPECT_TRUE(Matrix22(1, 2, 3, 4) != Matrix22(4, 3, 2, 1));

  Matrix22 matrix1{1, 2, 3, 4};
  EXPECT_TRUE(matrix1[0][0] == 1.0f);
  EXPECT_TRUE(matrix1[0][1] == 2.0f);
  EXPECT_TRUE(matrix1[1][0] == 3.0f);
  EXPECT_TRUE(matrix1[1][1] == 4.0f);

  matrix1.set(5, 6, 7, 8);
  matrix1 = Matrix22(1, 2, 3, 4);
  EXPECT_TRUE(matrix1 == Matrix22(1, 2, 3, 4));

  EXPECT_TRUE(Matrix22(1, 2, 3, 4) + Matrix22(5, 6, 7, 8) == Matrix22(6, 8, 10, 12));
  EXPECT_TRUE(Matrix22(1, 2, 3, 4) - Matrix22(5, 6, 7, 8) == Matrix22(-4, -4, -4, -4));

  Matrix22 matrix2{1, 2, 3, 4};
  Matrix22 matrix3{1, 2, 3, 4};
  matrix2 += Matrix22(5, 6, 7, 8);
  matrix3 -= Matrix22(5, 6, 7, 8);
  EXPECT_TRUE(matrix2 == Matrix22(6, 8, 10, 12));
  EXPECT_TRUE(matrix3 == Matrix22(-4, -4, -4, -4));

  EXPECT_TRUE(Matrix22(1, 2, 3, 4) * 2.0f == Matrix22(2, 4, 6, 8));
  EXPECT_TRUE(2.0f * Matrix22(1, 2, 3, 4) == Matrix22(2, 4, 6, 8));
  EXPECT_TRUE(Matrix22(1, 2, 3, 4) * Matrix22(5, 6, 7, 8) == Matrix22(19, 22, 43, 50));

  matrix2.set(1, 2, 3, 4);
  matrix2 *= 2.0f;
  EXPECT_TRUE(matrix2 == Matrix22(2, 4, 6, 8));

  Vector2 vector1{1, 2};
  matrix2.set(1, 2, 3, 4);
  matrix3.set(5, 6, 7, 8);
  Vector2 vector2 = matrix2 * vector1;
  Matrix22 matrix4 = matrix2 * matrix3;
  EXPECT_TRUE(vector2 == Vector2(5, 11));
  EXPECT_TRUE(matrix4 == Matrix22(19, 22, 43, 50));

  Matrix22 matrix5(-1, -2, -3, -4);
  Matrix22 matrix6 = -matrix5;
  EXPECT_TRUE(matrix6 == Matrix22(1, 2, 3, 4));
}

TEST(Matrix22, GetColumn) {
  Matrix22 matrix{1, 2, 3, 4};
  Vector2 column1;
  Vector2 column2;
  column1 = matrix.getColumn(0);
  column2 = matrix.getColumn(1);
  EXPECT_TRUE(column1 == Vector2(1, 3));
  EXPECT_TRUE(column2 == Vector2(2, 4));
}

TEST(Matrix22, GetRow) {
  Matrix22 matrix{1, 2, 3, 4};
  Vector2 column1;
  Vector2 column2;
  column1 = matrix.getRow(0);
  column2 = matrix.getRow(1);
  EXPECT_TRUE(column1 == Vector2(1, 2));
  EXPECT_TRUE(column2 == Vector2(3, 4));
}

TEST(Matrix22, GetTranspose) {
  Matrix22 matrix1{1, 2, 3, 4};
  Matrix22 matrix2 = matrix1.getTranspose();
  EXPECT_TRUE(matrix2.getRow(0) == Vector2(1, 3));
  EXPECT_TRUE(matrix2.getRow(1) == Vector2(2, 4));
}

TEST(Matrix22, GetDeterminant) {
  Matrix22 matrix{1, 2, 3, 4};
  float determinant = matrix.getDeterminant();
  EXPECT_TRUE(approximateEqual(determinant, -2.0f));
}

TEST(Matrix22, GetIdentity) {
  Matrix22 matrix1{1, 2, 3, 4};
  Matrix22 matrix2;
  matrix2 = matrix1.getIdentity();
  EXPECT_TRUE(matrix2.getRow(0) == Vector2(1, 0));
  EXPECT_TRUE(matrix2.getRow(1) == Vector2(0, 1));
}

TEST(Matrix22, GetZero) {
  Matrix22 matrix1{1, 2, 3, 4};
  Matrix22 matrix2{2, 4, 6, 8};
  matrix2 = matrix1.getZero();
  EXPECT_TRUE(matrix2.getRow(0) == Vector2(0, 0));
  EXPECT_TRUE(matrix2.getRow(1) == Vector2(0, 0));
}

TEST(Matrix22, Abs) {
  Matrix22 matrix1{-1, -2, 3, 4};
  Matrix22 matrix2 = abs(matrix1);
  EXPECT_TRUE(matrix1 == Matrix22(1, 2, 3, 4));
}