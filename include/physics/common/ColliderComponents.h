#ifndef PHYSICS_COLLIDER_COMPONENTS_H
#define PHYSICS_COLLIDER_COMPONENTS_H

#include <physics/mathematics/Transform.h>
#include <physics/common/Entity.h>
#include <physics/collections/Map.h>
#include <physics/collections/DynamicArray.h>
#include <physics/common/Components.h>
#include <physics/dynamics/Material.h>
#include <physics/collision/AABB.h>

namespace physics {

/* Forward declarations */
class MemoryHandler;
class Shape;
class Collider;

class ColliderComponents : public Components {

  public:
    /* -- Nested Classes -- */

    /* Collider component data */
    struct ColliderComponent {
      /* -- Attributes -- */

      /* Body entity in ECS */
      Entity bodyEntity;

      /* Pointer to collider */
      Collider* collider;

      /* Local bounds */
      AABB bounds;

      /* Local to body transform */
      const Transform& transformLocalBody;

      /* Local to world transform */
      const Transform& transformLocalWorld;

      /* Material */
      const Material& material;

      /* Pointer to collision shape */
      Shape* shape;

      /* Collision category */
      unsigned short collisionCategory;

      /* Collision filter */
      unsigned short collisionFilter;

      /* -- Methods -- */

      /* Constructor */
      ColliderComponent(Entity bodyEntity,
                        Collider* collider,
                        AABB bounds,
                        const Transform& transformLocalBody,
                        const Transform& transformLocalWorld,
                        const Material& material,
                        Shape* shape,
                        unsigned short collisionCategory,
                        unsigned short collisionFilter) :
                        bodyEntity(bodyEntity),
                        collider(collider),
                        bounds(bounds),
                        transformLocalBody(transformLocalBody),
                        transformLocalWorld(transformLocalWorld),
                        material(material),
                        shape(shape),
                        collisionCategory(collisionCategory),
                        collisionFilter(collisionFilter) {}
    };

  private:
    /* -- Attributes -- */

    /* Body entities */
    Entity* mBodyEntities;

    /* Collider entities */
    Entity* mColliderEntities;

    /* Pointers to colliders */
    Collider** mColliders;

    /* Array of collider broad phase identifiers */
    int32* mBroadPhaseIdentifiers;

    /* Array of Local to body transforms */
    Transform* mTransformsLocalBody;

    /* Array of local to world transforms */
    Transform* mTransformsLocalWorld;

    /* Array of material properties */
    Material* mMaterials;

    /* Array of pointers to collider shapes */
    Shape** mShapes;

    /* Array of collider collision categories */
    unsigned short* mCollisionCategories;

    /* Array of collider collision filters */
    unsigned short* mCollisionFilters;

    /* Array of overlapping pairs for each collider */
    DynamicArray<uint64>* mOverlapPairs;

    /* Array of collider shape size change statuses */
    bool* mHasSizeChanged;

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
    ColliderComponents(MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~ColliderComponents() override = default;

    /* Insert component */
    void insertComponent(Entity entity, bool isSleeping, const ColliderComponent& component);

    /* Get body entity */
    Entity getBodyEntity(Entity entity) const;

    /* Get pointer to collider */
    Collider* getCollider(Entity entity) const;

    /* Get broad phase identifier */
    int32 getBroadPhaseIdentifier(Entity entity) const;

    /* Set broad phase identifier */
    void setBroadPhaseIdentifier(Entity entity, int32 identifier);

    /* Get local to body transform */
    const Transform& getTransformLocalBody(Entity entity) const;

    /* Set local to body transform */
    void setTransformLocalBody(Entity entity, const Transform& transform);

    /* Get local to world transform */
    const Transform& getTransformLocalWorld(Entity entity) const;

    /* Set local to world transform */
    void setTransformLocalWorld(Entity entity, const Transform& transform);

    /* Get material */
    Material& getMaterial(Entity entity) const;

    /* Set material */
    void setMaterial(Entity entity, const Material& material);

    /* Get pointer to shape */
    Shape* getShape(Entity entity) const;

    /* Get collision category */
    unsigned short getCollisionCategory(Entity entity) const;

    /* Set collision category */
    void setCollisionCategory(Entity entity, unsigned short collisionCategory);

    /* Get Collision filter */
    unsigned short getCollisionFilter(Entity entity) const;

    /* Set Collision filter */
    void setCollisionFilter(Entity entity, unsigned short collisionFilter);

    /* Get array of overlap pairs */
    DynamicArray<uint64>& getOverlapPairs(Entity entity);

    /* Get size change status */
    bool getHasSizeChanged(Entity entity) const;

    /* Set size change status */
    void setHasSizeChanged(Entity entity, bool hasSizeChanged);

    /* -- Friends -- */
    friend class BroadPhase;
    friend class OverlapPairs;
    friend class Body;
    friend class CollisionDetection;
    friend class ContactSolver;
    friend class Dynamics;
};

}

#endif