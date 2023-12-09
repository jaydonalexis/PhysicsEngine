#include <physics/common/ColliderComponents.h>
#include <physics/collision/Collider.h>
#include <physics/common/EntityHandler.h>
#include <cassert>

using namespace physics;

/* Debug */
/* rm ? */
/* Constructor */
ColliderComponents::ColliderComponents(MemoryHandler& memoryHandler) :
                            Components(memoryHandler,
                                       sizeof(Entity) +
                                       sizeof(Entity) +
                                       sizeof(Collider*) +
                                       sizeof(int32) +
                                       sizeof(Transform) +
                                       sizeof(Transform) +
                                       sizeof(Material) +
                                       sizeof(Shape*) +
                                       sizeof(unsigned short) +
                                       sizeof(unsigned short) +
                                       sizeof(DynamicArray<uint64>) +
                                       sizeof(bool)) {
  /* Allocate memory for component data */
  allocate(NUM_INIT);
}

/* Allocate memory for components */
void ColliderComponents::allocate(uint32 numComponents) {
  assert(numComponents > mNumAllocatedComponents);
  /* Size for single component data multiplied the number of components */
  const size_t totalSize = mComponentByteSize * numComponents;
  /* Allocate memory for the component data */
  void* raw = mMemoryHandler.allocate(totalSize);
  assert(raw);

  /* Create pointers to component data */
  Entity* bodyEntities = static_cast<Entity*>(raw);
  Entity* colliderEntities = reinterpret_cast<Entity*>(bodyEntities + numComponents);
  Collider** colliders = reinterpret_cast<Collider**>(colliderEntities + numComponents);
  int32* broadPhaseIdentifiers = reinterpret_cast<int32*>(colliders + numComponents);
  Transform* transformsLocalBody = reinterpret_cast<Transform*>(broadPhaseIdentifiers + numComponents);
  Transform* transformsLocalToWorld = reinterpret_cast<Transform*>(transformsLocalBody + numComponents);
  Material* materials = reinterpret_cast<Material*>(transformsLocalToWorld + numComponents);
  Shape** shapes = reinterpret_cast<Shape**>(materials + numComponents);
  unsigned short* collisionCategories = reinterpret_cast<unsigned short*>(shapes + numComponents);
  unsigned short* collisionFilters = reinterpret_cast<unsigned short*>(collisionCategories + numComponents);
  DynamicArray<uint64>* overlapPairs = reinterpret_cast<DynamicArray<uint64>*>(collisionFilters + numComponents);
  bool* hasSizeChanged = reinterpret_cast<bool*>(overlapPairs + numComponents);

  /* Copy previous data to our new buffers */
  if(mNumComponents) {
    /* Copy component data from the previous buffer to the new buffer */
    memcpy(bodyEntities, mBodyEntities, mNumComponents * sizeof(Entity));
    memcpy(colliderEntities, mColliderEntities, mNumComponents * sizeof(Entity));
    memcpy(colliders, mColliders, mNumComponents * sizeof(Collider*));
    memcpy(broadPhaseIdentifiers, mBroadPhaseIdentifiers, mNumComponents * sizeof(int32));
    memcpy(transformsLocalBody, mTransformsLocalBody, mNumComponents * sizeof(Transform));
    memcpy(transformsLocalToWorld, mTransformsLocalWorld, mNumComponents * sizeof(Transform));
    memcpy(materials, mMaterials, mNumComponents * sizeof(Material));
    memcpy(shapes, mShapes, mNumComponents * sizeof(Shape*));
    memcpy(collisionCategories, mCollisionCategories, mNumComponents * sizeof(unsigned short));
    memcpy(collisionFilters, mCollisionFilters, mNumComponents * sizeof(unsigned short));
    memcpy(overlapPairs, mOverlapPairs, mNumComponents * sizeof(DynamicArray<uint64>));
    memcpy(hasSizeChanged, mHasSizeChanged, mNumComponents * sizeof(bool));

    /* Release the memory occupied by the previous buffers */
    mMemoryHandler.free(mData, mComponentByteSize * mNumAllocatedComponents);
  }

  /* Assign pointers to component attributes */
  mData = raw;
  mBodyEntities = bodyEntities;
  mColliderEntities = colliderEntities;
  mColliders = colliders;
  mBroadPhaseIdentifiers = broadPhaseIdentifiers;
  mTransformsLocalBody = transformsLocalBody;
  mTransformsLocalWorld = transformsLocalToWorld;
  mMaterials = materials;
  mShapes = shapes;
  mCollisionCategories = collisionCategories;
  mCollisionFilters = collisionFilters;
  mOverlapPairs = overlapPairs;
  mHasSizeChanged = hasSizeChanged;
  mNumAllocatedComponents = numComponents;
}

/* Erase component at provided index */
void ColliderComponents::eraseComponent(uint32 index) {
  Components::eraseComponent(index);
  assert(mEntityComponentMap[mColliderEntities[index]] == index);
  mEntityComponentMap.remove(mColliderEntities[index]);

  /* Destroy individual components */
  mColliderEntities[index].~Entity();
  mBodyEntities[index].~Entity();
  mColliders[index] = nullptr;
  mTransformsLocalBody[index].~Transform();
  mTransformsLocalWorld[index].~Transform();
  mMaterials[index].~Material();
  mShapes[index] = nullptr;
  mOverlapPairs[index].~DynamicArray<uint64>();
}

/* Move component from one index to another */
void ColliderComponents::moveComponent(uint32 source, uint32 destination) {
  const Entity entity = mColliderEntities[source];

  /* Copy source -> destination */
  new (mBodyEntities + destination) Entity(mBodyEntities[source]);
  new (mColliderEntities + destination) Entity(mColliderEntities[source]);
  mColliders[destination] = mColliders[source];
  new (mBroadPhaseIdentifiers + destination) int32(mBroadPhaseIdentifiers[source]);
  new (mTransformsLocalBody + destination) Transform(mTransformsLocalBody[source]);
  new (mTransformsLocalWorld + destination) Transform(mTransformsLocalWorld[source]);
  new (mMaterials + destination) Material(mMaterials[source]);
  mShapes[destination] = mShapes[source];
  mCollisionCategories[destination] = mCollisionCategories[source];
  mCollisionFilters[destination] = mCollisionFilters[source];
  new (mOverlapPairs + destination) DynamicArray<uint64>(mOverlapPairs[source]);
  mHasSizeChanged[destination] = mHasSizeChanged[source];

  /* Destroy source */
  eraseComponent(source);
  assert(!mEntityComponentMap.contains(entity));
  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, destination));
  assert(mEntityComponentMap[mColliderEntities[destination]] == destination);
}

/* Debug */
/* Swap components */
void ColliderComponents::swapComponents(uint32 first, uint32 second) {
  /* Copy first component's data */
  Entity firstBodyEntity(mBodyEntities[first]);
  Entity firstColliderEntity(mColliderEntities[first]);
  Collider* firstCollider = mColliders[first];
  int32 firstBroadPhaseIdentifier(mBroadPhaseIdentifiers[first]);
  Transform firstTransformLocalBody(mTransformsLocalBody[first]);
  Transform firstTransformLocalWorld(mTransformsLocalWorld[first]);
  Material firstMaterial(mMaterials[first]);
  Shape* firstShape = mShapes[first];
  unsigned short firstCollisionCategory = mCollisionCategories[first];
  unsigned short firstCollisionFilter = mCollisionFilters[first];
  DynamicArray<uint64> firstOverlapPair(mOverlapPairs[first]);
  bool firstHasSizeChanged = mHasSizeChanged[first];

  /* Destroy first component */
  eraseComponent(first);
  /* Move second's component data to first's location */
  moveComponent(second, first);

  /* Construct first's component data at second */
  new (mBodyEntities + second) Entity(firstBodyEntity);
  new (mColliderEntities + second) Entity(firstColliderEntity);
  mColliders[second] = firstCollider;
  new (mBroadPhaseIdentifiers + second) int32(firstBroadPhaseIdentifier);
  new (mTransformsLocalBody + second) Transform(firstTransformLocalBody);
  new (mTransformsLocalWorld + second) Transform(firstTransformLocalWorld);
  new (mMaterials + second) Material(firstMaterial);
  mShapes[second] = firstShape;
  mCollisionCategories[second] = firstCollisionCategory;
  mCollisionFilters[second] = firstCollisionFilter;
  new (mOverlapPairs + second) DynamicArray<uint64>(firstOverlapPair);
  mHasSizeChanged[second] = firstHasSizeChanged;

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(firstColliderEntity, second));
  assert(mEntityComponentMap[mColliderEntities[first]] == first);
  assert(mEntityComponentMap[mColliderEntities[second]] == second);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Insert component */
void ColliderComponents::insertComponent(Entity entity, bool isSleeping, const ColliderComponent& component) {
  /* Find insert index */
  uint32 insertIndex = computeInsertIndex(isSleeping);

  /* Insert component data */
  new (mBodyEntities + insertIndex) Entity(component.bodyEntity);
  new (mColliderEntities + insertIndex) Entity(entity);
  mColliders[insertIndex] = component.collider;
  new (mBroadPhaseIdentifiers + insertIndex) int32(-1);
  new (mTransformsLocalBody + insertIndex) Transform(component.transformLocalBody);
  new (mTransformsLocalWorld + insertIndex) Transform(component.transformLocalWorld);
  new (mMaterials + insertIndex) Material(component.material);
  mShapes[insertIndex] = component.shape;
  mCollisionCategories[insertIndex] = component.collisionCategory;
  mCollisionFilters[insertIndex] = component.collisionFilter;
  new (mOverlapPairs + insertIndex) DynamicArray<uint64>(mMemoryHandler);
  mHasSizeChanged[insertIndex] = false;

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, insertIndex));
  mNumComponents++;
  assert(mSleepingStartIndex <= mNumComponents);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Get body entity */
Entity ColliderComponents::getBodyEntity(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mBodyEntities[mEntityComponentMap[entity]];
}

/* Get pointer to collider */
Collider* ColliderComponents::getCollider(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mColliders[mEntityComponentMap[entity]];
}

/* Get broad phase identifier */
int32 ColliderComponents::getBroadPhaseIdentifier(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mBroadPhaseIdentifiers[mEntityComponentMap[entity]];
}

/* Set broad phase identifier */
void ColliderComponents::setBroadPhaseIdentifier(Entity entity, int32 identifier) {
  assert(mEntityComponentMap.contains(entity));
  mBroadPhaseIdentifiers[mEntityComponentMap[entity]] = identifier;
}

/* Get local to body transform */
const Transform& ColliderComponents::getTransformLocalBody(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mTransformsLocalBody[mEntityComponentMap[entity]];
}

/* Set local to body transform */
void ColliderComponents::setTransformLocalBody(Entity entity, const Transform& transform) {
  assert(mEntityComponentMap.contains(entity));
  mTransformsLocalBody[mEntityComponentMap[entity]] = transform;
}

/* Get local to world transform */
const Transform& ColliderComponents::getTransformLocalWorld(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mTransformsLocalWorld[mEntityComponentMap[entity]];
}

/* Set local to world transform */
void ColliderComponents::setTransformLocalWorld(Entity entity, const Transform& transform) {
  assert(mEntityComponentMap.contains(entity));
  mTransformsLocalWorld[mEntityComponentMap[entity]] = transform;
}

/* Get material */
Material& ColliderComponents::getMaterial(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mMaterials[mEntityComponentMap[entity]];
}

/* Set material */
void ColliderComponents::setMaterial(Entity entity, const Material& material) {
  assert(mEntityComponentMap.contains(entity));
  mMaterials[mEntityComponentMap[entity]] = material;
}

/* Get pointer to shape */
Shape* ColliderComponents::getShape(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mShapes[mEntityComponentMap[entity]];
}

/* Get collision category */
unsigned short ColliderComponents::getCollisionCategory(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mCollisionCategories[mEntityComponentMap[entity]];
}

/* Set collision category */
void ColliderComponents::setCollisionCategory(Entity entity, unsigned short collisionCategory) {
  assert(mEntityComponentMap.contains(entity));
  mCollisionCategories[mEntityComponentMap[entity]] = collisionCategory;
}

/* Get Collision filter */
unsigned short ColliderComponents::getCollisionFilter(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mCollisionFilters[mEntityComponentMap[entity]];
}

/* Set Collision filter */
void ColliderComponents::setCollisionFilter(Entity entity, unsigned short collisionFilter) {
  assert(mEntityComponentMap.contains(entity));
  mCollisionFilters[mEntityComponentMap[entity]] = collisionFilter;
}

/* Get array of overlap pairs */
DynamicArray<uint64>& ColliderComponents::getOverlapPairs(Entity entity) {
  assert(mEntityComponentMap.contains(entity));
  return mOverlapPairs[mEntityComponentMap[entity]];
}

/* Get size change status */
bool ColliderComponents::getHasSizeChanged(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mHasSizeChanged[mEntityComponentMap[entity]];
}

/* Set size change status */
void ColliderComponents::setHasSizeChanged(Entity entity, bool hasSizeChanged) {
  assert(mEntityComponentMap.contains(entity));
  mHasSizeChanged[mEntityComponentMap[entity]] = hasSizeChanged;
}
