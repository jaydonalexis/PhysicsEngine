#ifndef PHYSICS_ENTITY_HANDLER_H
#define PHYSICS_ENTITY_HANDLER_H

#include <physics/Configuration.h>
#include <physics/collections/DynamicArray.h>
#include <physics/collections/List.h>
#include <physics/common/Entity.h>
#include <cassert>

namespace physics {

class EntityHandler {

  private:
    /* -- Attributes -- */

    /* Generations array */
    DynamicArray<uint8> mGenerations;

    /* Indices array */
    List<uint32> mFreeIndices;

  public:
    /* -- Methods -- */

    /* Constructor */
    EntityHandler(MemoryHandler& memoryHandler);

    /* Create entity */
    Entity createEntity();

    /* Delete entity */
    void deleteEntity(Entity entity);

    /* Return whether the entity has been deleted or not */
    bool entityValid(Entity entity) const;
};

/* Constructor */
inline EntityHandler::EntityHandler(MemoryHandler& memoryHandler) : mGenerations(memoryHandler), mFreeIndices(memoryHandler) {}

/* Create entity */
inline Entity EntityHandler::createEntity() {
  uint32 index;

  /* There are enough free indices to start using them */
  if(mFreeIndices.size() > Entity::NUM_MIN_FREE_INDICES) {
    /* Reuse an index */
    index = mFreeIndices.getHead()->data;
    mFreeIndices.removeFront();
  }
  else {
    /* New index being used */
    mGenerations.add(0);
    index = static_cast<uint32>(mGenerations.size()) - 1;
    assert(index < uint32(1) << Entity::NUM_INDEX_BITS);
  }

  return Entity(index, mGenerations[index]);
}

/* Delete entity */
inline void EntityHandler::deleteEntity(Entity entity) {
  const uint32 index = entity.getIndex();

  /* Generation is increased for when this index is reused by another created entity */
  mGenerations[index]++;

  /* Index is now free */
  mFreeIndices.addBack(index);
}

inline bool EntityHandler::entityValid(Entity entity) const {
  return mGenerations[entity.getIndex()] == entity.getGeneration();
}

}

#endif;