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
    enum class HandlerType {Linear, ObjectPool, FreeList, Vanilla, Primary};

    /* -- Methods -- */

    /* Constructor */
    MemoryFactory(MemoryHandler* primaryMemoryHandler, size_t initSize = 0);

    /* Destructor */
    ~MemoryFactory() = default;

    /* Dynamically allocate memory using a specific memory handler */
    void* allocate(HandlerType handlerType, size_t size);

    /* Free dynamically allocated memory */
    void free(HandlerType handlerType, void* ptr, size_t size);

    /* Get Linear handler */
    LinearMemoryHandler& getLinearMemoryHandler();

    /* Get Object Pool handler */
    ObjectPoolMemoryHandler& getObjectPoolMemoryHandler();

    /* Get Free List handler */
    FreeListMemoryHandler& getFreeListMemoryHandler();

    /* Get Vanilla handler */
    VanillaMemoryHandler& getVanillaMemoryHandler();

    /* Reset memory handler if applicable */
    void reset(HandlerType handlerType);
};

/* Dynamically allocate memory using a specific memory handler */
inline void* MemoryFactory::allocate(HandlerType handlerType, size_t size) {
  switch(handlerType) {
    case HandlerType::Linear: return mLinearyMemoryHandler.allocate(size);
    case HandlerType::ObjectPool: return mObjectPoolMemoryHandler.allocate(size);
    case HandlerType::FreeList: return mFreeListMemoryHandler.allocate(size);
    case HandlerType::Vanilla: return mVanillaMemoryHandler.allocate(size);
    case HandlerType::Primary: return mPrimaryMemoryHandler->allocate(size);
    default: return nullptr;
  }
}

/* Free dynamically allocated memory */
inline void MemoryFactory::free(HandlerType handlerType, void* ptr, size_t size) {
  switch(handlerType) {
    case HandlerType::Linear: mLinearyMemoryHandler.free(ptr, size); break;
    case HandlerType::ObjectPool: mObjectPoolMemoryHandler.free(ptr, size); break;
    case HandlerType::FreeList: mFreeListMemoryHandler.free(ptr, size); break;
    case HandlerType::Vanilla: mVanillaMemoryHandler.free(ptr, size); break;
    case HandlerType::Primary: mPrimaryMemoryHandler->free(ptr, size); break;
    default: return;
  }
}

/* Get Linear handler */
inline LinearMemoryHandler& MemoryFactory::getLinearMemoryHandler() {
  return mLinearyMemoryHandler;
}

/* Get Object Pool handler */
inline ObjectPoolMemoryHandler& MemoryFactory::getObjectPoolMemoryHandler() {
  return mObjectPoolMemoryHandler;
}

/* Get Free List handler */
inline FreeListMemoryHandler& MemoryFactory::getFreeListMemoryHandler() {
  return mFreeListMemoryHandler;
}

/* Get Vanilla memory handler */
inline VanillaMemoryHandler& MemoryFactory::getVanillaMemoryHandler() {
  return mVanillaMemoryHandler;
}

/* Reset memory handler if applicable */
inline void MemoryFactory::reset(HandlerType handlerType) {
  switch(handlerType) {
    case HandlerType::Linear: mLinearyMemoryHandler.reset(); break;
    case HandlerType::ObjectPool: break;
    case HandlerType::FreeList: break;
    case HandlerType::Vanilla: break;
    case HandlerType::Primary: break;
  }
}

}

#endif