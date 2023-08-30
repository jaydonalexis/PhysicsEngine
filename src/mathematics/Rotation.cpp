#include <physics/mathematics/Rotation.h>
#include <physics/mathematics/Vector2.h>

using namespace physics;

/* Get the x axis */
Vector2 Rotation::getX() const {
  return Vector2(c, s);
}

Vector2 Rotation::getY() const {
  return Vector2(-s, c);
}