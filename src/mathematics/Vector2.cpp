#include <physics/mathematics/Vector2.h>

using namespace physics;

/* Get the associated unit vector of the current vector */
Vector2 Vector2::getUnitVector() const {
  float len = length();

  if(len < MACHINE_EPSILON) {
    return *this;
  }

  float lenInverse = float(1.0) / len;
  return Vector2(x * lenInverse, y * lenInverse);
}