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
  const float MACHINE_EPSILON = std::numeric_limits<float>::epsilon();

  /* PI constant */
  constexpr float PI = float(3.141592653589);

  constexpr float PI_MUL_2 = float(6.283185307180);

  constexpr float PI_DIV_2 = float(1.570796326795);
}

#endif