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

    /* Restitution constant */
    float mRestitution;

    /* Debug */
    /* Friction constant */
    float mFriction;

    /* -- Methods -- */

    /* Constructor */
    Material(float frictionConstant, float restitutionConstant, float density = 1.0f);

  public:
    /* -- Methods -- */

    /* Get the density of the material */
    float getDensity() const;

    /* Set the density of the material */
    void setDensity(float density);

    /* Get the restitution constant of the material */
    float getRestitution() const;

    /* Set the restitution constant of the material */
    void setRestitution(float restitution);

    /* Get the friction constant of the material */
    float getFriction() const;

    /* Set the friction constant of the material */
    void setFriction(float friction);

    /* -- Friends -- */
    friend class Collider;
    friend class Body;
};

/* Constructor */
inline Material::Material(float frictionConstant, float restitutionConstant, float density) : mFriction(frictionConstant), mRestitution(restitutionConstant), mDensity(density) {} 

/* Get the density of the material */
inline float Material::getDensity() const {
  return mDensity;
}


/* Set the density of the material */
inline void Material::setDensity(float density) {
  assert(density > 0.0f);
  mDensity = density;
}

/* Get the restitution constant of the material */
inline float Material::getRestitution() const {
  return mRestitution;
}

/* Set the restitution constant of the material */
inline void Material::setRestitution(float restitution) {
  assert(restitution >= 0.0f && restitution <= 1.0f);
  mRestitution = restitution;
}

/* Get the friction constant of the material */
inline float Material::getFriction() const {
  return mFriction;
}

/* Set the friction constant of the material */
inline void Material::setFriction(float friction) {
  assert(friction >= 0.0f);
  mFriction = friction;
}

}

#endif