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

    /* Constructor with parameters */
    Rotation(float newS, float newC);

    /* Set angle of rotation in radians */
    void set(float angle);

    /* Set to identity rotation */
    void setZero();

    /* Get the angle of rotation in radians */
    float get() const;

    /* Get the x axis */
    Vector2 getX() const;

    /* Get the y axis */
    Vector2 getY() const;

    /* Get identity rotation */
    static Rotation getZero();

    /* Overloaded equality operator */
    bool operator==(const Rotation& orientation) const;

    /* Overloaded inequality operator */
    bool operator!=(const Rotation& orientation) const;

    /* Overloaded operator for multiplication with assignment */
    Rotation& operator*=(const Rotation& orientation);

    /* -- Friends -- */

    friend Rotation operator*(const Rotation& orientation1, const Rotation& orientation2);
    friend Vector2 operator*(const Rotation& orientation, const Vector2& vector);
    friend Rotation transposeMultiply(const Rotation& orientation1, const Rotation& orientation2);
    friend Vector2 transposeMultiply(const Rotation& orientation, const Vector2& vector);
};

/* Constructor */
inline Rotation::Rotation() : s(0), c(1) {}

/* Constructor with parameters */
inline Rotation::Rotation(float angle) : s(sinf(angle)), c(cosf(angle)) {}

/* Constructor with parameters */
inline Rotation::Rotation(float newS, float newC) : s(newS), c(newC) {}

/* Set angle of rotation in radians */
inline void Rotation::set(float angle) {
  s = sinf(angle);
  c = cosf(angle);
}

/* Set to identity rotation */
inline void Rotation::setZero() {
  s = 0;
  c = 1;
}

/* Get the angle of rotation in radians */
inline float Rotation::get() const {
  return atan2f(s, c);
}

/* Get identity rotation */
inline Rotation Rotation::getZero() {
  return Rotation(0, 1);
}

/* Overloaded equality operator */
inline bool Rotation::operator==(const Rotation& orientation) const {
  return approximateEqual(s, orientation.s) && approximateEqual(c, orientation.c);
}

/* Overloaded inequality operator */
inline bool Rotation::operator!=(const Rotation& orientation) const {
  return !(*this == orientation);
}

/* Overloaded operator for multiplication with assignment */
inline Rotation& Rotation::operator*=(const Rotation& orientation) {
  return Rotation(c * orientation.s - s * orientation.c, c * orientation.c + s * orientation.s);
}

/* Overloaded operator for multiplication between two given orientations */
inline Rotation operator*(const Rotation& orientation1, const Rotation& orientation2) {
  return Rotation(orientation1.c * orientation2.s - orientation1.s * orientation2.c,
                  orientation1.c * orientation2.s + orientation1.s * orientation2.c);
}

/* Transpose multiplication between two given orientations */
inline Rotation transposeMultiply(const Rotation& orientation1, const Rotation& orientation2) {
  return Rotation(orientation1.c * orientation2.s - orientation1.s * orientation2.c,
                  orientation1.c * orientation2.c + orientation1.s * orientation2.s);
}

}

#endif