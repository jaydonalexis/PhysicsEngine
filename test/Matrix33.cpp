#include "UnitTests.h"

#include <physics/mathematics/Matrix33.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Matrix33, Constructor) {
  Matrix33 matrix1;
  EXPECT_TRUE(approximateEqual(matrix1[0][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[0][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[1][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[1][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[2][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[2][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix1[2][2], 0.0f));

  Matrix33 matrix2{1};
  EXPECT_TRUE(approximateEqual(matrix2[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][2], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][2], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][2], 1.0f));

  Matrix33 matrix3{1, 2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_TRUE(approximateEqual(matrix3[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix3[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix3[0][2], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix3[1][0], 4.0f));
  EXPECT_TRUE(approximateEqual(matrix3[1][1], 5.0f));
  EXPECT_TRUE(approximateEqual(matrix3[1][2], 6.0f));
  EXPECT_TRUE(approximateEqual(matrix3[2][0], 7.0f));
  EXPECT_TRUE(approximateEqual(matrix3[2][1], 8.0f));
  EXPECT_TRUE(approximateEqual(matrix3[2][2], 9.0f));
}

TEST(Matrix33, CopyConstructor) {
  Matrix33 matrix1{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix33 matrix2(matrix1);
  EXPECT_TRUE(approximateEqual(matrix2[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix2[0][2], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][0], 4.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][1], 5.0f));
  EXPECT_TRUE(approximateEqual(matrix2[1][2], 6.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][0], 7.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][1], 8.0f));
  EXPECT_TRUE(approximateEqual(matrix2[2][2], 9.0f));
}

TEST(Matrix33, SetValus) {
  Matrix33 matrix;
  matrix.set(1);
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][2], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][2], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][2], 1.0f));

  matrix.set(1, 2, 3, 4, 5, 6, 7, 8, 9);
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 2.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][2], 3.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 4.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 5.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][2], 6.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][0], 7.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][1], 8.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][2], 9.0f));

  matrix.setZero();
  EXPECT_TRUE(approximateEqual(matrix[0][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][2], 0.0f));

  matrix.setIdentity();
  EXPECT_TRUE(approximateEqual(matrix[0][0], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[0][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][1], 1.0f));
  EXPECT_TRUE(approximateEqual(matrix[1][2], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][0], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][1], 0.0f));
  EXPECT_TRUE(approximateEqual(matrix[2][2], 1.0f));
}

TEST(Matrix33, Operators) {
  EXPECT_TRUE(Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) == Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9));
  EXPECT_TRUE(Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) != Matrix33(9, 8, 7, 6, 5, 4, 3, 2, 1));
  
  Matrix33 matrix1{1, 2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_TRUE(matrix1[0][0] == 1.0f);
  EXPECT_TRUE(matrix1[0][1] == 2.0f);
  EXPECT_TRUE(matrix1[0][2] == 3.0f);
  EXPECT_TRUE(matrix1[1][0] == 4.0f);
  EXPECT_TRUE(matrix1[1][1] == 5.0f);
  EXPECT_TRUE(matrix1[1][2] == 6.0f);
  EXPECT_TRUE(matrix1[2][0] == 7.0f);
  EXPECT_TRUE(matrix1[2][1] == 8.0f);
  EXPECT_TRUE(matrix1[2][2] == 9.0f);

  matrix1.set(10, 11, 12, 13, 14, 15, 16, 17, 18);
  matrix1 = Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9);
  EXPECT_TRUE(matrix1 == Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9));

  EXPECT_TRUE(Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) + Matrix33(9, 8, 7, 6, 5, 4, 3, 2, 1) ==
              Matrix33(10, 10, 10, 10, 10, 10, 10, 10, 10));
  EXPECT_TRUE(Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) - Matrix33(9, 8, 7, 6, 5, 4, 3, 2, 1) ==
              Matrix33(-8, -6, -4, -2, 0, 2, 4, 6, 8));

  Matrix33 matrix2{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix33 matrix3{1, 2, 3, 4, 5, 6, 7, 8, 9};
  matrix2 += Matrix33(9, 8, 7, 6, 5, 4, 3, 2, 1);
  matrix3 -= Matrix33(9, 8, 7, 6, 5, 4, 3, 2, 1);
  EXPECT_TRUE(matrix2 == Matrix33(10, 10, 10, 10, 10, 10, 10, 10, 10));
  EXPECT_TRUE(matrix3 == Matrix33(-8, -6, -4, -2, 0, 2, 4, 6, 8));

  EXPECT_TRUE(Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) * 2.0f== Matrix33(2, 4, 6, 8, 10, 12, 14, 16, 18));
  EXPECT_TRUE(2.0f * Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9) == Matrix33(2, 4, 6, 8, 10, 12, 14, 16, 18));

  matrix2.set(1, 2, 3, 4, 5, 6, 7, 8, 9);
  matrix2 *= 2.0f;
  EXPECT_TRUE(matrix2 == Matrix33(2, 4, 6, 8, 10, 12, 14, 16, 18));

  Vector3 vector1{1, 2, 3};
  matrix3.set(1, 2, 3, 4, 5, 6, 7, 8, 9);
  Vector3 vector2 = matrix3 * vector1;
  EXPECT_TRUE(vector2 == Vector3(14, 32, 50));

  Matrix33 matrix4(-1, -2, -3, -4, -5, -6, -7, -8, -9);
  Matrix33 matrix5 = -matrix4;
  EXPECT_TRUE(matrix5 == Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9));
}

TEST(Matrix33, GetColumn) {
  Matrix33 matrix{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Vector3 column1;
  Vector3 column2;
  Vector3 column3;
  column1 = matrix.getColumn(0);
  column2 = matrix.getColumn(1);
  column3 = matrix.getColumn(2);
  EXPECT_TRUE(column1 == Vector3(1, 4, 7));
  EXPECT_TRUE(column2 == Vector3(2, 5, 8));
  EXPECT_TRUE(column3 == Vector3(3, 6, 9));
}

TEST(Matrix33, GetRow) {
  Matrix33 matrix{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Vector3 row1;
  Vector3 row2;
  Vector3 row3;
  row1 = matrix.getRow(0);
  row2 = matrix.getRow(1);
  row3 = matrix.getRow(2);
  EXPECT_TRUE(row1 == Vector3(1, 2, 3));
  EXPECT_TRUE(row2 == Vector3(4, 5, 6));
  EXPECT_TRUE(row3 == Vector3(7, 8, 9));
}

TEST(Matrix33, GetTranspose) {
  Matrix33 matrix1{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix33 matrix2 = matrix1.getTranspose();
  EXPECT_TRUE(matrix2.getRow(0) == Vector3(1, 4, 7));
  EXPECT_TRUE(matrix2.getRow(1) == Vector3(2, 5, 8));
  EXPECT_TRUE(matrix2.getRow(2) == Vector3(3, 6, 9));
}

TEST(Matrix33, GetDeterminant) {
  Matrix33 matrix{1, 2, 3, 4, 5, 9, 8, 7, 6};
  float determinant = matrix.getDeterminant();
  EXPECT_TRUE(approximateEqual(determinant, 27.0f));
}

TEST(Matrix33, GetIdentity) {
  Matrix33 matrix1{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix33 matrix2;
  matrix2 = matrix1.getIdentity();
  EXPECT_TRUE(matrix2.getRow(0) == Vector3(1, 0, 0));
  EXPECT_TRUE(matrix2.getRow(1) == Vector3(0, 1, 0));
  EXPECT_TRUE(matrix2.getRow(2) == Vector3(0, 0, 1));
}

TEST(Matrix33, GetZero) {
  Matrix33 matrix1{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Matrix33 matrix2;
  matrix2 = matrix1.getZero();
  EXPECT_TRUE(matrix2.getRow(0) == Vector3(0, 0, 0));
  EXPECT_TRUE(matrix2.getRow(1) == Vector3(0, 0, 0));
  EXPECT_TRUE(matrix2.getRow(2) == Vector3(0, 0, 0));
}