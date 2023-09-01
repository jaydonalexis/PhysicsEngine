#include <physics/mathematics/Vector3.h>

using namespace physics;

/* Get the associated unit vector of the current vector */
Vector3 Vector3::getUnitVector() const {
  float len = length();

  if(len < MACHINE_EPSILON) {
    return *this;
  }

  float lenInverse = float(1.0) / len;
  
  return Vector3(x * lenInverse, y * lenInverse, z * lenInverse);
}