#ifndef PHYSICS_COLLIDER_H
#define PHYSICS_COLLIDER_H

#include <physics/collision/Shape.h>
#include <physics/dynamics/Body.h>
#include <physics/dynamics/Material.h>

namespace physics {

/* Forward declarations */
class MemoryHandler;
class MemoryStrategy;

/* Enables a body to collide in the physics world */
class Collider {

  protected:
    /* -- Attributes -- */

    /* Identifier */
    Entity mEntity;

    /* Owning body */
    Body* mBody;

    /* Memory strategy */
    MemoryStrategy& mMemoryStrategy;

    /* -- Methods -- */

    /* Collider's representative collision shape has been changed */
    void setShapeSizeChanged(bool shapeSizeChanged);

  public:
    /* -- Methods -- */

    /* Constructor */
    Collider(Entity entity, Body* body, MemoryStrategy& memoryStrategy);

    /* Destructor */
    ~Collider() = default;

    /* Deleted copy constructor */
    Collider(const Collider& collider) = delete;

    /* Deleted assignment operator */
    Collider& operator=(const Collider& collider) = delete;

    /* Get collider's identifier */
    Entity getEntity() const;

    /* Return a pointer to the collider's representative collision shape */
    Shape* getShape();

    /* Return a const pointer to the collider's representative collision shape */
    const Shape* getShape() const;

    /* Return a pointer to the collider's owning body */
    Body* getBody() const;

    /* Get the local to owning body transform */
    const Transform& getTransformLocalBody() const;

    /* Set the local to owning body transform */
    void setTransformLocalBody(const Transform& transform);

    /* Get the local to world transform */
    const Transform& getTransformLocalWorld() const;

    /* Return world space AABB of the collider */
    const AABB getAABB() const;

    /* Query whether the collider overlaps with the given AABB */
    bool testOverlap(const AABB& aabb);

    /* Query whether a point is inside the collider's representative collision shape */
    bool testPoint(const Vector2& point);

    /* Debug */
    /* Get collision category */
    unsigned short getCollisionCategory() const;

    /* Debug */
    /* Set collision category */
    void setCollisionCategory(unsigned short collisionCategory);

    /* Get collision filter */
    unsigned short getCollisionFilter() const;

    /* Set collision filter */
    void setCollisionFilter(unsigned short collisionCompatibility);

    /* Get broad phase identifier */
    int32 getBroadPhaseIdentifier() const;

    /* Return a reference to the collider's material */
    Material& getMaterial();

    /* Set the collider's material */
    void setMaterial(const Material& material);

    /* -- Friends -- */
    
    friend class Shape;
};

}

#endif