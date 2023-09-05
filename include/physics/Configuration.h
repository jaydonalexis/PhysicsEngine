#ifndef PHYSICS_CONFIGURATION_H
#define	PHYSICS_CONFIGURATION_H

/* Libraries */
#include <limits>
#include <cfloat>
#include <utility>
#include <sstream>
#include <string>
// #include <physics/containers/Pair.h>

namespace physics {
  /* Type Definitions */

  using uint = unsigned int;
  using uchar = unsigned char;
  using ushort = unsigned short;
  using luint = long unsigned int;

  using int8 = std::int8_t;
  using uint8 = std::uint8_t;
  using int16 = std::int16_t;
  using uint16 = std::uint16_t;
  using int32 = std::int32_t;
  using uint32 = std::uint32_t;
  using int64 = std::int64_t;
  using uint64 = std::uint64_t;

  struct Entity;
  // using bodyPair = Pair<Entity, Entity>;

  /* Constants */

  /* Smallest float value */
  const float FLOAT_SMALLEST = - std::numeric_limits<float>::max();

  /* Largest float value */
  const float FLOAT_LARGEST = std::numeric_limits<float>::max();

  /* Machine epsilon */
  const float FLOAT_EPSILON = std::numeric_limits<float>::epsilon() * 1e+1f;

  /* PI constants */
  constexpr float PI = 3.141592653589f;
  constexpr float PI_MUL_2 = 6.283185307180f;
  constexpr float PI_DIV_2 = 1.570796326795f;
  constexpr float PI_DIV_4 = 0.785398163397f;
  constexpr float PI_DIV_8 = 0.392699081699f;
}

#endif