#ifndef PHSYICS_ROTATION_H
#define PHYSICS_ROTATION_H

#include <physics/mathematics/MathCommon.h>

namespace physics {

/* Forward declaration */
struct Vector2;

struct Rotation {

  public:
    /* -- Attributes -- */

    /* sin component */
    float s;

    /* cosine component */
    float c;

    /* -- Methods -- */

    /* Constructor */
    Rotation();

    /* Constructor with parameters */
    Rotation(float angle);

    /* Set angle of rotation in radians */
    void set(float angle);

    /* Set to zero rotation */
    void setZero();

    /* Get the angle of rotation in radians */
    float get() const;

    /* Get the x axis */
    Vector2 getX() const;

    /* Get the y axis */
    Vector2 getY() const;
};

/* Constructor */
inline Rotation::Rotation() : s(0.0f), c(1.0f) {}

/* Constructor with parameters */
inline Rotation::Rotation(float angle) : s(sinf(angle)), c(cosf(angle)) {}

/* Set angle of rotation in radians */
inline void Rotation::set(float angle) {
  s = sinf(angle);
  c = cosf(angle);
}

/* Set to identity rotation */
inline void Rotation::setZero() {
  s = 0.0f;
  c = 1.0f;
}

/* Get the angle of rotation in radians */
inline float Rotation::get() const {
  return atan2f(s, c);
}

}

#endif