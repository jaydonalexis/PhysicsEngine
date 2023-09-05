#include <physics/mathematics/Vector3.h>

using namespace physics;

/* Get the associated unit vector of the current vector */
Vector3 Vector3::getUnitVector() const {
  float len = length();

  if(len < FLOAT_EPSILON) {
    return *this;
  }

  float lenInverse = 1.0f / len;
  
  return Vector3(x * lenInverse, y * lenInverse, z * lenInverse);
}