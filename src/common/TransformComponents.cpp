#include <physics/common/TransformComponents.h>
#include <physics/common/EntityHandler.h>
#include <cassert>

using namespace physics;

/* Constructor */
TransformComponents::TransformComponents(MemoryHandler& memoryHandler) :
                    Components(memoryHandler,
                               sizeof(Entity) +
                               sizeof(Transform)) {
  /* Allocate memory for component data */
  allocate(NUM_INIT);
}

/* Allocate memory for components */
void TransformComponents::allocate(uint32 numComponents) {
  assert(numComponents > mNumAllocatedComponents);
  /* Size for single component data multiplied the number of components */
  const size_t totalSize = mComponentByteSize * numComponents;
  /* Allocate memory for the component data */
  void* raw = mMemoryHandler.allocate(totalSize);
  assert(raw);

  /* Create pointers to component data */
  Entity* bodyEntities = static_cast<Entity*>(raw);
  Transform* transforms = reinterpret_cast<Transform*>(bodyEntities + numComponents);

  /* Copy previous data to our new buffers */
  if(mNumComponents)  {
    /* Copy component data from the previous buffer to the new buffer */
    memcpy(bodyEntities, mBodyEntities, mNumComponents * sizeof(Entity));
    memcpy(transforms, mTransforms, mNumComponents * sizeof(Transform));

    /* Release the memory occupied by the previous buffers */
    mMemoryHandler.free(mData, mComponentByteSize * mNumAllocatedComponents);
  }

  /* Assign pointers to component attributes */
  mData = raw;
  mBodyEntities = bodyEntities;
  mTransforms = transforms;
  mNumAllocatedComponents = numComponents;
}

/* Erase component at the provided index */
void TransformComponents::eraseComponent(uint32 index) {
  Components::eraseComponent(index);
  assert(mEntityComponentMap[mBodyEntities[index]] == index);
  mEntityComponentMap.remove(mBodyEntities[index]);

  /* Destroy individual components */
  mBodyEntities[index].~Entity();
  mTransforms[index].~Transform();
}

/* Move component from one index to another */
void TransformComponents::moveComponent(uint32 source, uint32 destination) {
  const Entity entity = mBodyEntities[source];

  /* Copy source -> destination */
  new (mBodyEntities + destination) Entity(mBodyEntities[source]);
  new (mTransforms + destination) Transform(mTransforms[source]);

  /* Destroy source */
  eraseComponent(source);
  assert(!mEntityComponentMap.contains(entity));
  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, destination));
  assert(mEntityComponentMap[mBodyEntities[destination]] == destination);
}

/* Swap components */
void TransformComponents::swapComponents(uint32 first, uint32 second) {
  /* Copy first component's data */
  Entity firstEntity(mBodyEntities[first]);
  Transform firstTransform(mTransforms[first]);

  /* Destroy first component */
  eraseComponent(first);
  /* Move second's component data to first's location */
  moveComponent(second, first);

  /* Construct first's component data at second */
  new (mBodyEntities + second) Entity(firstEntity);
  new (mTransforms + second) Transform(firstTransform);

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(firstEntity, second));
  assert(mEntityComponentMap[mBodyEntities[first]] == first);
  assert(mEntityComponentMap[mBodyEntities[second]] == second);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Insert component */
void TransformComponents::insertComponent(Entity entity, bool isSleeping, const TransformComponent& component) {
  /* Find insert index */
  uint32 insertIndex = computeInsertIndex(isSleeping);

  /* Insert component data */
  new (mBodyEntities + insertIndex) Entity(entity);
  new (mTransforms + insertIndex) Transform(component.transform);

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, insertIndex));
  mNumComponents++;
  assert(mSleepingStartIndex <= mNumComponents);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Get transform */
Transform& TransformComponents::getTransform(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mTransforms[mEntityComponentMap[entity]];
}

/* Set transform */
void TransformComponents::setTransform(Entity entity, const Transform& transform) {
  assert(mEntityComponentMap.contains(entity));
  mTransforms[mEntityComponentMap[entity]] = transform;
}