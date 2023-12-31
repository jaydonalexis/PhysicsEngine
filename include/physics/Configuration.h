#ifndef PHYSICS_CONFIGURATION_H
#define	PHYSICS_CONFIGURATION_H

/* Libraries */
#include <limits>
#include <cfloat>
#include <utility>
#include <sstream>
#include <string>
#include <iostream>
#include <physics/collections/Pair.h>

#define NOT_USED(x) ((void)(x))
#define LOG_FILE "../log/log.txt"

namespace physics {
  /* -- Type Definitions -- */

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

  /* -- Constants -- */

  /* Smallest float value */
  const float FLOAT_SMALLEST = - std::numeric_limits<float>::max();

  /* Largest float value */
  const float FLOAT_LARGEST = std::numeric_limits<float>::max();

  /* Machine epsilon */
  const float FLOAT_EPSILON = std::numeric_limits<float>::epsilon() * 1e+1f;

  /* PI constants */
  constexpr float PI = 3.141592653589f;

  /* Number of shape types */
  constexpr uint8 NUM_SHAPE_TYPES = 2;

  /* Minimum polygon vertices */
  constexpr uint8 MIN_POLYGON_VERTICES = 3;

  /* Maximum polygon vertices */
  constexpr uint8 MAX_POLYGON_VERTICES = 8;

  /* Debug world scale */
  /* Dynamic tree fat AABB inflation */
  constexpr float DYNAMIC_TREE_FAT_AABB_INFLATION = 0.1f;

  /* Dynamic tree fat AABB inflation */
  constexpr float DYNAMIC_TREE_FAT_AABB_MULTIPLIER = 4.0f;

  /* Debug world scale */
  /* A small length used as a collision and constraint tolerance */
  constexpr float LINEAR_SLOP = 0.005f;

  /* A small angle used as a collision and constraint tolerance */
  constexpr float ANGULAR_SLOP = 2.0f / 180.0f * PI;

  /* Polygon radius */
  constexpr float POLYGON_RADIUS = 0.0f;

  /* Quick hull algorithm weld tolerance */
  constexpr float QUICK_HULL_WELD_TOLERANCE = 16.0f * LINEAR_SLOP * LINEAR_SLOP;

  /* Maximum number of points in a contact manifold */
  constexpr uint8 MAX_MANIFOLD_POINTS = 2;

  /* Block solver maximum condition number */
  constexpr float BLOCK_SOLVER_MAX_CONDITION = 1000.0f;

  /* Constant which controls how fast overlap is resolved */
  constexpr float BAUMGARTE = 0.2f;

  /* Maximum position correction when solving position constraints */
  constexpr float MAX_LINEAR_CORRECTION = 0.2f;

  /* Maximum translation */
  constexpr float MAX_TRANSLATION = 4.0f;

  /* Maximum rotation */
  constexpr float MAX_ROTATION = (0.5f * PI) * (0.5f * PI);
}

#endif