#include <physics/mathematics/Matrix22.h>

using namespace physics;

Matrix22 Matrix22::getInverse() const {
  float determinant = getDeterminant();

  assert(abs(determinant) > MACHINE_EPSILON);

  float determinantInverse = float(1.0) / determinant;

  Matrix22 temp(mRows[1][1], -mRows[0][1], 
                -mRows[1][0], mRows[0][0]);

  return determinantInverse * temp;
}