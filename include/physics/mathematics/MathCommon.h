#ifndef PHYSICS_MATH_COMMON_H
#define PHYSICS_MATH_COMMON_H

#include <cassert>
#include <cmath>
#include <physics/Configuration.h>

namespace physics {

  template <typename T>
  inline T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
  }

  inline uint32 nextPowerOfTwo(uint32 value) {
    value |= (value >> 1);
    value |= (value >> 2);
    value |= (value >> 4);
    value |= (value >> 8);
    value |= (value >> 16);
    return value + 1;
  }

  inline bool isPowerOfTwo(uint32 value) {
    return value > 0 && (value & (value - 1)) == 0;
  }

  inline bool approximateEqual(float a, float b, float epsilon = FLOAT_EPSILON) {
    return std::abs(a - b) < epsilon;
  }

}

#endif