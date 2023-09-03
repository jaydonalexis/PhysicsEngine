#ifndef PHYSICS_MATH_COMMON_H
#define PHYSICS_MATH_COMMON_H

#include <cassert>
#include <cmath>
#include <physics/Configuration.h>

namespace physics {
  inline bool approximateEqual(float a, float b, float epsilon = MACHINE_EPSILON) {
    return std::abs(a - b) < epsilon;
  }
}

#endif