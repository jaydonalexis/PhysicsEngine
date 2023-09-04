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
    bool operator==(const Transform& orientation) const;

    /* Overloaded inequality operator */
    bool operator!=(const Transform& orientation) const;

    /* Overloaded multiplication operator with assignment */
    Transform& operator*=(const Transform& orientation);

    /* -- Friends -- */
    
    friend Transform operator*(const Transform& transform1, const Transform& transform2);
    friend Vector2 operator*(const Transform& transform, const Vector2& vector);

};

/* Constructor */
inline Transform::Transform() : mPosition(Vector2(0.0, 0.0)), mOrientation(Rotation::getZero()) {}

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

}

#endif
