#ifndef PHYSICS_TRANSFORM_H
#define PHYSICS_TRANSFORM_H

#include <cassert>
#include <physics/mathematics/Vector2.h>
#include <physics/mathematics/Rotation.h>

namespace physics {

class Transform {

  private:
    /* -- Attributes -- */

    /* Position */
    Vector2 mPosition;

    /* Orientation */
    Rotation mOrientation;

  public:
    /* -- Methods -- */

    /* Constructor */
    Transform();

    /* Constructor with parameters */
    Transform(const Vector2& position, const Rotation& orientation);

    /* Set position of the transform */
    void setPosition(const Vector2& position);

    /* Set orientation of the transform */
    void setOrientation(const Rotation& orientation);

    /* Set transform to identity transform */
    void setIdentity();

    /* Get position of the transform */
    const Vector2& getPosition() const;

    /* Get orientation of the transform */
    const Rotation& getOrientation() const;

    /* Get identity transform */
    static Transform getIdentity();

    /* Overloaded equality operator */
    bool operator==(const Transform& transform) const;

    /* Overloaded inequality operator */
    bool operator!=(const Transform& transform) const;

    /* Overloaded multiplication operator with assignment */
    Transform& operator*=(const Transform& transform);

    /* -- Friends -- */
    
    friend Transform operator*(const Transform& transform1, const Transform& transform2);
    friend Vector2 operator*(const Transform& transform, const Vector2& vector);
    friend Transform transposeMultiply(const Transform& transform1, const Transform& transform2);
    friend Vector2 transposeMultiply(const Transform& transform, const Vector2& vector);

};

/* Constructor */
inline Transform::Transform() : mPosition(Vector2(0.0f, 0.0f)), mOrientation(Rotation::getZero()) {}

/* Constructor with parameters */
inline Transform::Transform(const Vector2& position, const Rotation& orientation) : mPosition(position), mOrientation(orientation) {}

/* Set position of the transform */
inline void Transform::setPosition(const Vector2& position) {
  mPosition = position;
}

/* Set orientation of the transform */
inline void Transform::setOrientation(const Rotation& orientation) {
  mOrientation = orientation;
}

/* Set transform to identity transform */
inline void Transform::setIdentity() {
  mPosition = Vector2(0, 0);
  mOrientation = Rotation::getZero();
}

/* Get position of the transform */
inline const Vector2& Transform::getPosition() const {
  return mPosition;
}

/* Get orientation of the transform */
inline const Rotation& Transform::getOrientation() const {
  return mOrientation;
}

/* Get identity transform */
inline Transform Transform::getIdentity() {
  return Transform(Vector2(0, 0), Rotation::getZero());
}

/* Overloaded equality operator */
inline bool Transform::operator==(const Transform& transform) const {
  return mPosition == transform.mPosition && mOrientation == transform.mOrientation;
}

/* Overloaded inequality operator */
inline bool Transform::operator!=(const Transform& transform) const {
  return !(*this == transform);
}

/* Overloaded multiplication operator with assignment */
inline Transform& Transform::operator*=(const Transform& transform) {
  mPosition = (mOrientation * transform.mPosition) + mPosition;
  mOrientation = mOrientation * transform.mOrientation;
  return *this;
}

/* Overloaded operator for multiplication between two given transforms */
inline Transform operator*(const Transform& transform1, const Transform& transform2) {
  return Transform((transform1.mOrientation * transform2.mPosition) + transform1.mPosition, transform1.mOrientation * transform2.mOrientation);
}

/* Overloaded operator for multiplication of a given transform with a given vector */
inline Vector2 operator*(const Transform& transform, const Vector2& vector) {
  return Vector2((transform.mOrientation.c * vector.x - transform.mOrientation.s * vector.y) + transform.mPosition.x,
                 (transform.mOrientation.s * vector.x + transform.mOrientation.c * vector.y) + transform.mPosition.y);
}

/* Transpose multiplication between two given transforms */
inline Transform transposeMultiply(const Transform& transform1, const Transform& transform2) {
  return Transform(transform1.mOrientation * (transform2.mPosition - transform1.mPosition), transform1.mOrientation * transform2.mOrientation);
}

/* Transpose multiplication between a given transform and a given vector */
inline Vector2 transposeMultiply(const Transform& transform, const Vector2& vector) {
  float px = vector.x - transform.mPosition.x;
  float py = vector.y - transform.mPosition.y;
  return Vector2(transform.mOrientation.c * px + transform.mOrientation.s * py,
                 -transform.mOrientation.s * px + transform.mOrientation.c * py);
}

}

#endif
