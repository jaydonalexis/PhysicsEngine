#include <physics/mathematics/Matrix22.h>

using namespace physics;

Matrix22 Matrix22::getInverse() const {
  float determinant = getDeterminant();

  assert(std::abs(determinant) > FLOAT_EPSILON);

  float determinantInverse = 1.0f / determinant;

  Matrix22 temp(mRows[1][1], -mRows[0][1], 
                -mRows[1][0], mRows[0][0]);

  return determinantInverse * temp;
}