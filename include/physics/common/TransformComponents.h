#ifndef PHYSICS_TRANSFORM_COMPONENTS_H
#define PHYSICS_TRANSFORM_COMPONENTS_H

#include <physics/mathematics/Transform.h>
#include <physics/common/Entity.h>
#include <physics/collections/Map.h>
#include <physics/common/Components.h>

namespace physics {

/* Forward declarations */
class MemoryHandler;

/* Entity transforms */
class TransformComponents : public Components {
  
  public:
    /* -- Nested Classes -- */

    /* Transform component data */
    struct TransformComponent {

      public:
        /* -- Attributes -- */

        /* Entity transform */
        const Transform& transform;

        /* -- Methods -- */  

        /* Constructor */
        TransformComponent(const Transform& transform) : transform(transform) {}
    };

  private:
    /* -- Attributes -- */

    /* Body entities */
    Entity* mBodyEntities;

    /* Array of transforms */
    Transform* mTransforms;

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
    TransformComponents(MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~TransformComponents() override = default;

    /* Insert component */
    void insertComponent(Entity entity, bool isSleeping, const TransformComponent& component);

    /* Get transform */
    Transform& getTransform(Entity entity) const;

    /* Set transform */
    void setTransform(Entity entity, const Transform& transform);

    /* -- Friends -- */
    friend class ContactSolver;
};

}

#endif