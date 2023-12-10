#ifndef PHYSICS_MATH_COMMON_H
#define PHYSICS_MATH_COMMON_H

#include <cassert>
#include <cmath>
#include <physics/Configuration.h>

namespace physics {

  /* Clamp the provided value between the given low and high thresholds */
  template <typename T>
  inline T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
  }

  /* Return the square of the provided value */
  template <typename T>
  inline T square(T value) {
    return value * value;
  }

  /* Return the next power of two that is larger than the given value */
  inline uint64 nextPowerOfTwo(uint64 value) {
    value |= (value >> 1);
    value |= (value >> 2);
    value |= (value >> 4);
    value |= (value >> 8);
    value |= (value >> 16);
    return value + 1;
  }

  /* Query whether the given value is a power of two */
  inline bool isPowerOfTwo(uint64 value) {
    return value > 0 && (value & (value - 1)) == 0;
  }

  /* Query whether the two provided decimal values are approximately equal */
  inline bool approximateEqual(float a, float b, float epsilon = FLOAT_EPSILON) {
    return std::abs(a - b) < epsilon;
  }

  /* Get a unique integer from the two provided numbers */
  inline uint64 getElegantPair(uint32 firstNumber, uint32 secondNumber) {
    if(firstNumber != std::max(firstNumber, secondNumber)) {
      return getElegantPair(secondNumber, firstNumber);
    }

    return static_cast<uint64>(firstNumber * firstNumber + firstNumber + secondNumber);
  }

}

#endif