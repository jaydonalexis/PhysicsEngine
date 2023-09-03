#include <physics/mathematics/Matrix33.h>

using namespace physics;

Matrix33 Matrix33::getInverse() const {
  float determinant = getDeterminant();

  assert(abs(determinant) > MACHINE_EPSILON);

  float determinantInverse = float(1.0) / determinant;

  Matrix33 temp((mRows[1][1] * mRows[2][2] - mRows[2][1] * mRows[1][2]), -(mRows[0][1] * mRows[2][2] - mRows[2][1] * mRows[0][2]), (mRows[0][1] * mRows[1][2] - mRows[0][2] * mRows[1][1]),
               -(mRows[1][0] * mRows[2][2] - mRows[2][0] * mRows[1][2]), (mRows[0][0] * mRows[2][2] - mRows[2][0] * mRows[0][2]), -(mRows[0][0] * mRows[1][2] - mRows[1][0] * mRows[0][2]),
                (mRows[1][0] * mRows[2][1] - mRows[2][0] * mRows[1][1]), -(mRows[0][0] * mRows[2][1] - mRows[2][0] * mRows[0][1]), (mRows[0][0] * mRows[1][1] - mRows[0][1] * mRows[1][0]));

  return determinantInverse * temp;
}