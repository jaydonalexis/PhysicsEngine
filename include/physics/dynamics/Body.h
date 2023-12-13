#ifndef PHYSICS_BODY_H
#define PHYSICS_BODY_H

#include <physics/common/Entity.h>
#include <physics/collision/AABB.h>
#include <physics/mathematics/Transform.h>
#include <physics/mathematics/Math.h>
#include <cassert>

namespace physics {

/* Forward declarations */
class Collider;
class Shape;
class World;
enum class BodyType;

/* Debug */
/* Remove virtual */
/* Non-compressible object of constant mass */
class Body {

  protected:
    /* -- Attributes -- */

    /* Identifier of the entity */
    Entity mEntity;

    /* Reference to physics world */
    World& mWorld;

    /* -- Methods -- */

    /* Remove all of the overlapping pairs that the body is involved in */
    void resetOverlapPairs();

    /* Compute the center of mass using the body's colliders */
    Vector2 computeMassProperties() const;

    /* Remove all collision shapes */
    void removeColliders();

    /* Update state in broad phase */
    void updateBroadPhase() const;

    /* Check the collision shapes of the body for collision in broadphase */
    void checkBroadPhaseCollision() const;

  public:
    /* -- Methods -- */

    /* Constructor */
    Body(World& world, Entity entity);

    /* Destructor */
    ~Body() = default;

    /* Deleted copy constructor */
    Body(const Body& body) = delete;

    /* Deleted assignment operator */
    Body& operator=(const Body& body) = delete;

    /* Get the entity of the body */
    Entity getEntity() const;

    /* Create new collider and add it to the body */
    Collider* addCollider(Shape* shape, const Transform& transform);

    /* Remove a collider from the body */
    void removeCollider(Collider* collider);

    /* Get a constant pointer to a given collider of the body */
    const Collider* getCollider(uint32 index) const;

    /* Return a pointer to a given collider of the body */
    Collider* getCollider(uint32 index);

    /* Get number of colliders */
    uint32 getNumColliders() const;

    /* Query whether a point is inside the body */
    bool testPoint(const Vector2& pointWorld) const;

    /* Query whether the body overlaps with the given AABB */
    bool testOverlap(const AABB& aabb) const;

    /* Get the body's AABB by merging all of its colliders' AABBs */
    AABB getAABB() const;

    /* Get current position and orientation */
    const Transform& getTransform() const;

    /* Set current position and orientation */
    void setTransform(const Transform& transform);

    /* Get mass of the body */
    float getMass() const;

    /* Set mass of the body */
    void setMass(float mass);

    /* Get linear velocity of the body */
    Vector2 getLinearVelocity() const;

    /* Set linear velocity of the body */
    void setLinearVelocity(const Vector2& linearVelocity);

    /* Get angular speed of the body */
    float getAngularSpeed() const;

    /* Set angular speed of the body */
    void setAngularSpeed(float angularSpeed);

    /* Get linear damping of the body */
    float getLinearDamping() const;

    /* Set linear damping of the body */
    void setLinearDamping(float linearDamping);

    /* Get angular damping of the body */
    float getAngularDamping() const;

    /* Set angular damping of the body */
    void setAngularDamping(float angularDamping);

    /* Get inertia of the body */
    float getInertia() const;

    /* Set inertia of the body */
    void setInertia(float inertia);

    /* Get local center of mass */
    const Vector2& getCenterOfMassLocal() const;

    /* Set local center of mass */
    void setCenterOfMassLocal(const Vector2& centerOfMass);
    
    /* Set local space center of mass using the body's colliders */
    void setCenterOfMassLocalWithColliders();

    /* Set local space inertia using the body's colliders */
    void setLocalInertiaUsingColliders();

    /* Set the mass using the body's colliders */
    void setMassUsingColliders();

    /* Set all mass properties of the body using the body's colliders */
    void setMassPropertiesUsingColliders();

    /* Get type of the body */
    BodyType getType() const;

    /* Set type of the body */
    void setType(BodyType type);

    /* Query whether gravity is enabled for this body */
    bool isGravityEnabled() const;

    /* Set whether gravity is enabled for this body */
    void setIsGravityEnabled(bool isGravityEnabled);

    /* Query whether this body is allowed to sleep */
    bool isAllowedToSleep() const;

    /* Set whether this body is allowed to sleep */
    void setIsAllowedToSleep(bool isAllowedToSleep);

    /* Query whether the body is sleeping */
    bool isSleeping() const;

    /* Set whether the body is sleeping */
    void setIsSleeping(bool isSleeping);

    /* Apply world force to body at world point */
    void applyForce(const Vector2& force, const Vector2& point);

    /* Apply world force to body at world center of mass position */
    void applyForceToCenter(const Vector2& force);

    /* Apply world torque to body */
    void applyTorque(float torque);

    /* Clear the total force acting on the body */
    void clearForces();

    /* Clear the total torque acting on the body */
    void clearTorques();

    /* Get the total force acting on the body */
    const Vector2& getForce() const;

    /* Get the total torque acting on the body */
    float getTorque() const;

    /* -- Friends -- */

    friend class Collider;
    friend class World;
};

}

#endif