#include <physics/mathematics/Rotation.h>
#include <physics/mathematics/Vector2.h>

using namespace physics;

/* Get the x axis */
Vector2 Rotation::getX() const {
  return Vector2(c, s);
}

/* Get the y axis */
Vector2 Rotation::getY() const {
  return Vector2(-s, c);
}

/* Overloaded operator for multiplication of a given orientation with a given vector */
inline Vector2 operator*(const Rotation& orientation, const Vector2& vector) {
  return Vector2(orientation.c * vector.x - orientation.s * vector.y,
                 orientation.s * vector.x + orientation.c * vector.y);
}

/* Transpose multiplication between a given orientation and a given vector */
inline Vector2 transposeMultiply(const Rotation& orientation, const Vector2& vector) {
  return Vector2(orientation.c * vector.x + orientation.s * vector.y,
                 -orientation.s * vector.x + orientation.c * vector.y);
}