#ifndef PHYSICS_MEMORY_FACTORY_H
#define PHYSICS_MEMORY_FACTORY_H

#include <physics/memory/Vanilla.h>
#include <physics/memory/Linear.h>
#include <physics/memory/ObjectPool.h>
#include <physics/memory/FreeList.h>

namespace physics {

/* Forward Declaration */
class MemoryHandler;

class MemoryFactory {

  private:
    /* -- Attributes -- */

    /* Debug */
    /* Primary memory handler */
    MemoryHandler* mPrimaryMemoryHandler;

    /* Vanilla/Default handler */
    VanillaMemoryHandler mVanillaMemoryHandler;

    /* Linear memory handler */
    LinearMemoryHandler mLinearyMemoryHandler;

    /* Object pool memory handler */
    ObjectPoolMemoryHandler mObjectPoolMemoryHandler;

    /* Free list memory handler */
    FreeListMemoryHandler mFreeListMemoryHandler;

  public:
    /* -- Nested Classes -- */

    /* Handler types */
    enum class HandlerType {Vanilla, Linear, ObjectPool, FreeList};

    /* -- Methods -- */

    /* Constructor */
    MemoryFactory(MemoryHandler* primaryMemoryHandler, size_t initSize = 0);

    /* Destructor */
    ~MemoryFactory() = default;

    /* Dynamically allocate memory using a specific memory handler */
    void* allocate(HandlerType handlerType, size_t size);

    /* Free dynamically allocated memory */
    void free(HandlerType handlerType, void* ptr, size_t size);

    /* Get Vanilla handler */
    VanillaMemoryHandler& getVanillaMemoryHandler();

    /* Get Linear handler */
    LinearMemoryHandler& getLinearMemoryHandler();

    /* Get Object Pool handler */
    ObjectPoolMemoryHandler& getObjectPoolMemoryHandler();

    /* Get Free List handler */
    FreeListMemoryHandler& getFreeListMemoryHandler();

    /* Reset memory handler if applicable */
    void reset();
};

}

#endif