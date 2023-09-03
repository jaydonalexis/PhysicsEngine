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
};

}

#endif
