#include <physics/mathematics/Vector2.h>

using namespace physics;

/* Get the associated unit vector of the current vector */
Vector2 Vector2::getUnitVector() const {
  float len = length();

  if(len < FLOAT_EPSILON) {
    return *this;
  }

  float lenInverse = 1.0f / len;
  
  return Vector2(x * lenInverse, y * lenInverse);
}