#ifndef PHYSICS_COMPONENTS_H
#define PHYSICS_COMPONENTS_H

#include <physics/Configuration.h>
#include <physics/common/Entity.h>
#include <physics/collections/Map.h>

namespace physics {

/* Forward declarations */
class EntityHandler;
class MemoryHandler;

class Components {

  protected:
    /* -- Constants -- */

    /* Initial number of components to allocate */
    const uint32 NUM_INIT = 10;

    /* -- Attributes -- */

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* Number of components */
    uint32 mNumComponents;

    /* Number of allocated components */
    uint32 mNumAllocatedComponents;

    /* Size of a component */
    size_t mComponentByteSize;

    /* Component data */
    void* mData;

    /* Entity-Component map */
    Map<Entity, uint32> mEntityComponentMap;

    /* First sleeping entity */
    uint32 mSleepingStartIndex;

    /* -- Methods -- */

    /* Allocate memory for components */
    virtual void allocate(uint32 numComponents)=0;

    /* Compute index to insert new component */
    uint32 computeInsertIndex(bool isSleeping);

    /* Erase component at the provided index */
    virtual void eraseComponent(uint32 index);

    /* Move component from one index to another */
    virtual void moveComponent(uint32 source, uint32 destination)=0;

    /* Swap components */
    virtual void swapComponents(uint32 first, uint32 second)=0;

  public:
    /* -- Methods -- */

    /* Constructor */
    Components(MemoryHandler& memoryHandler, size_t componentByteSize);

    /* Destructor */
    virtual ~Components();

    /* Remove component for given entity */
    void removeComponent(Entity entity);

    /* Contains component */
    bool containsComponent(Entity entity) const;

    /* Get component entity index */
    uint32 getComponentEntityIndex(Entity entity);

    /* Get the number of components */
    uint32 getNumComponents() const;

    /* Get the number of enabled components */
    uint32 getNumEnabledComponents() const;

    /* Query whether entity is disabled */
    bool getIsEntityDisabled(Entity entity) const;

    /* Set whether an entity is disabled */
    void setIsEntityDisabled(Entity entity, bool isDisabled);
};

}

#endif