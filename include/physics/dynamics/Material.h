#ifndef PHYSICS_MATERIAL_H
#define PHYSICS_MATERIAL_H

#include <physics/Configuration.h>
#include <cassert>

namespace physics {

/* Collider material properties */
class Material {

  private:
    /* -- Attributes -- */

    /* Density */
    float mDensity;

    /* Spring constant */
    float mSpring;

    /* Debug */
    /* Friction constant */
    float mFriction;

    /* -- Methods -- */

    /* Constructor */
    Material(float frictionConstant, float springConstant, float density = 1.0f);

  public:
    /* -- Methods -- */

    /* Get the density of the material */
    float getDensity() const;

    /* Set the density of the material */
    void setDensity(float density);

    /* Get the spring constant of the material */
    float getSpringConstant() const;

    /* Set the spring constant of the material */
    void setSpringConstant(float spring);

    /* Get the friction constant of the material */
    float getFrictionConstant() const;

    /* Set the friction constant of the material */
    void setFrictionConstant(float friction);

    /* -- Friends -- */
    friend class Collider;
    friend class Body;
};

inline float Material::getDensity() const {
  return mDensity;
}

inline void Material::setDensity(float density) {
  assert(density > 0.0f);
  mDensity = density;
}

inline float Material::getSpringConstant() const {
  return mSpring;
}

inline void Material::setSpringConstant(float spring) {
  assert(spring >= 0.0f && spring <= 1.0f);
  mSpring = spring;
}

inline float Material::getFrictionConstant() const {
  return mFriction;
}

inline void Material::setFrictionConstant(float friction) {
  assert(friction >= 0.0f);
  mFriction = friction;
}

}

#endif