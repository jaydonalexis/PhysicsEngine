#ifndef PHYSICS_BODY_COMPONENTS_H
#define PHYSICS_BODY_COMPONENTS_H

#include <physics/mathematics/Transform.h>
#include <physics/common/Entity.h>
#include <physics/collections/Map.h>
#include <physics/collections/DynamicArray.h>
#include <physics/common/Components.h>

namespace physics {

/* Forward declarations */
class MemoryHandler;
class Body;

/* Body type */
enum class BodyType {Static, Dynamic, Kinematic};

class BodyComponents : public Components {

  public:
    /* -- Nested Classes -- */

    /* Body component data */
    struct BodyComponent {

      public:
        /* -- Attributes -- */

        /* Pointer to body */
        Body* body;

        /* Body type */
        BodyType type;

        /* World position */
        const Vector2& worldPosition;

        /* -- Methods -- */

        /* Constructor */
        BodyComponent(Body* body, BodyType type, const Vector2& worldPosition) : body(body), type(type), worldPosition(worldPosition) {}
    };

  private:
    /* -- Attributes -- */

    /* Body entities */
    Entity* mBodyEntities;

    /* Pointers to bodies */
    Body** mBodies;

    /* Colliders of each body */
    DynamicArray<Entity>* mColliders;

    /* Array of sleep permissions */
    bool* mIsAllowedToSleep;

    /* Array of sleep statuses */
    bool* mIsSleeping;

    /* Array of sleep times */
    float* mSleepTimes;

    /* Array of body types */
    BodyType* mTypes;

    /* Array of linear velocities */
    Vector2* mLinearVelocities;

    /* Array of angular speeds */
    float* mAngularSpeeds;

    /* Array of external forces */
    Vector2* mForces;

    /* Array of external torques */
    float* mTorques;

    /* Array of Linear dampings */
    float* mLinearDampings;

    /* Array of angular dampings */
    float* mAngularDampings;

    /* Array of masses */
    float* mMasses;

    /* Array of inverse masses */
    float* mInverseMasses;

    /* Array of local rotational inertia */
    float* mInertias;

    /* Array of inverse local rotational inertia */
    float* mInverseInertias;

    /* Array of constrained linear velocities */
    Vector2* mLinearVelocitiesConstrained;

    /* Array of constrained angular speeds */
    float* mAngularSpeedsConstrained;

    /* Array of constrained positions */
    Vector2* mPositionsConstrained;

    /* Array of constrained orientations */
    Rotation* mOrientationsConstrained;

    /* Array of local center of masses */
    Vector2* mCentersOfMassLocal;

    /* Array of world center of masses */
    Vector2* mCentersOfMassWorld;

    /* Array of gravitational states */
    bool* mIsGravityEnabled;

    /* Array of island inclusion states */
    bool* mIsInIsland;

    /* Array of indices of contact pairs which the body is part of */
    DynamicArray<uint32>* mContactPairs;

    /* -- Methods -- */

    /* Allocate memory for components */
    virtual void allocate(uint32 numComponents) override;

    /* Erase component at the provided index */
    virtual void eraseComponent(uint32 index) override;

    /* Move component from one index to another */
    virtual void moveComponent(uint32 source, uint32 destination) override;

    /* Swap components */
    virtual void swapComponents(uint32 first, uint32 second) override;

  public:
    /* -- Methods -- */

    /* Constructor */
    BodyComponents(MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~BodyComponents() override = default;

    /* Insert component */
    void insertComponent(Entity entity, bool isSleeping, const BodyComponent& component);

    /* Get pointer to body */
    Body* getBody(Entity entity) const;

    /* Add a collider to the body */
    void addCollider(Entity bodyEntity, Entity colliderEntity);

    /* Remove a collider from the body */
    void removeCollider(Entity bodyEntity, Entity colliderEntity);

    /* Get the colliders of the body */
    const DynamicArray<Entity>& getColliders(Entity entity) const;

    /* Get sleep permission */
    bool getIsAllowedToSleep(Entity entity) const;

    /* Set sleep permission */
    void setIsAllowedToSleep(Entity entity, bool isAllowedToSleep);

    /* Query sleep status */
    bool getIsSleeping(Entity entity) const;

    /* Set sleep status */
    void setIsSleeping(Entity entity, bool isSleeping);

    /* Get sleep time */
    float getSleepTime(Entity entity) const;

    /* Set sleep time */
    void setSleepTime(Entity entity, float sleepTime);

    /* Get body type */
    BodyType getType(Entity entity) const;

    /* Set body type */
    void setType(Entity entity, BodyType type);

    /* Get linear velocity */
    const Vector2& getLinearVelocity(Entity entity) const;

    /* Set linear velocity */
    void setLinearVelocity(Entity entity, const Vector2& linearVelocity);

    /* Get angular speed */
    float getAngularSpeed(Entity entity) const;

    /* Set angular speed */
    void setAngularSpeed(Entity entity, float angularSpeed);

    /* Get external force */
    const Vector2& getForce(Entity entity) const;

    /* Set external force */
    void setForce(Entity entity, const Vector2& force);

    /* Get external torque */
    float getTorque(Entity entity) const;

    /* Set external torque */
    void setTorque(Entity entity, float torque);

    /* Get linear damping */
    float getLinearDamping(Entity entity) const;

    /* Set linear damping */
    void setLinearDamping(Entity entity, float linearDamping);

    /* Get angular damping */
    float getAngularDamping(Entity entity) const;

    /* Set angular damping */
    void setAngularDamping(Entity entity, float angularDamping);

    /* Get mass */
    float getMass(Entity entity) const;

    /* Set mass */
    void setMass(Entity entity, float mass);

    /* Get inverse mass */
    float getInverseMass(Entity entity) const;

    /* Set inverse mass */
    void setInverseMass(Entity entity, float inverseMass);

    /* Get rotational inertia */
    float getInertia(Entity entity) const;

    /* Set rotational inertia */
    void setInertia(Entity entity, float inertia);

    /* Get inverse rotational inertia */
    float getInverseInertia(Entity entity) const;

    /* Set inverse rotational inertia */
    void setInverseInertia(Entity entity, float inverseInertia);

    /* Set contrained linear velocity */
    void setConstrainedLinearVelocity(Entity entity, const Vector2& constrainedLinearVelocity);

    /* Set constrained angular speed */
    void setConstrainedAngularSpeed(Entity entity, float constrainedAngularSpeed);

    /* Get constrained position */
    Vector2& getConstrainedPosition(Entity entity) const;

    /* Set constrained position */
    void setConstrainedPosition(Entity entity, const Vector2& constrainedPosition);

    /* Get constrained orientation */
    Rotation& getConstrainedOrientation(Entity entity) const;

    /* Set constrained orientation */
    void setConstrainedOrientation(Entity entity, const Rotation& constrainedOrientation);

    /* Get local center of mass */
    const Vector2& getCenterOfMassLocal(Entity entity) const;

    /* Set local center of mass */
    void setCenterOfMassLocal(Entity entity, const Vector2& centerOfMassLocal);

    /* Get world center of mass */
    const Vector2& getCenterOfMassWorld(Entity entity) const;

    /* Set world center of mass */
    void setCenterOfMassWorld(Entity entity, const Vector2& centerOfMassWorld);

    /* Get gravitational state */
    bool getIsGravityEnabled(Entity entity) const;

    /* Set gravitational state */
    void setIsGravityEnabled(Entity entity, bool isGravityEnabled);

    /* Get island inclusion state */
    bool getIsInIsland(Entity entity) const;

    /* Set island inclusion state */
    void setIsInIsland(Entity, bool isInIsland);

    /* Add contact pairing index */
    void addContactPair(Entity entity, uint32 index);

    /* -- Friends -- */
    friend class World;
    friend class Body;
    friend class CollisionDetection;
    friend class Dynamics;
    friend class ContactSolver;
};

}

#endif