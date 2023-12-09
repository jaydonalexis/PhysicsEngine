#include <physics/common/BodyComponents.h>
#include <physics/dynamics/Body.h>
#include <physics/common/EntityHandler.h>
#include <cassert>

using namespace physics;

/* Debug */
/* rm? */
/* Constructor */
BodyComponents::BodyComponents(MemoryHandler& memoryHandler) :
                    Components(memoryHandler,
                               sizeof(Entity) +
                               sizeof(Body*) +
                               sizeof(DynamicArray<Entity>) +
                               sizeof(bool) +
                               sizeof(bool) +
                               sizeof(float) +
                               sizeof(BodyType) +
                               sizeof(Vector2) +
                               sizeof(float) +
                               sizeof(Vector2) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(float) +
                               sizeof(Vector2) +
                               sizeof(float) +
                               sizeof(Vector2) +
                               sizeof(Rotation) +
                               sizeof(Vector2) +
                               sizeof(Vector2) +
                               sizeof(bool) +
                               sizeof(bool) +
                               sizeof(DynamicArray<uint32>)) {
  /* Allocate memory for component data */
  allocate(NUM_INIT);
}

/* Allocate memory for components */
void BodyComponents::allocate(uint32 numComponents) {
  assert(numComponents > mNumAllocatedComponents);
  /* Size for single component data multiplied the number of components */
  const size_t totalSize = mComponentByteSize * numComponents;
  /* Allocate memory for the component data */
  void* raw = mMemoryHandler.allocate(totalSize);
  assert(raw);
  
  /* Create pointers to component data */
  Entity* bodyEntities = static_cast<Entity*>(raw);
  Body** bodies = reinterpret_cast<Body**>(bodyEntities + numComponents);
  DynamicArray<Entity>* colliders = reinterpret_cast<DynamicArray<Entity>*>(bodies + numComponents);
  bool* isAllowedToSleep = reinterpret_cast<bool*>(colliders + numComponents);
  bool* isSleeping = reinterpret_cast<bool*>(isAllowedToSleep + numComponents);
  float* sleepTimes = reinterpret_cast<float*>(isSleeping + numComponents);
  BodyType* types = reinterpret_cast<BodyType*>(sleepTimes + numComponents);
  Vector2* linearVelocities = reinterpret_cast<Vector2*>(types + numComponents);
  float* angularSpeeds = reinterpret_cast<float*>(linearVelocities + numComponents);
  Vector2* forces = reinterpret_cast<Vector2*>(angularSpeeds + numComponents);
  float* torques = reinterpret_cast<float*>(forces + numComponents);
  float* linearDampings = reinterpret_cast<float*>(torques + numComponents);
  float* angularDampings = reinterpret_cast<float*>(linearDampings + numComponents);
  float* masses = reinterpret_cast<float*>(angularDampings + numComponents);
  float* inverseMasses = reinterpret_cast<float*>(masses + numComponents);
  float* inertias = reinterpret_cast<float*>(inverseMasses + numComponents);
  float* inverseInertias = reinterpret_cast<float*>(inertias + numComponents);
  Vector2* linearVelocitiesConstrained = reinterpret_cast<Vector2*>(inverseInertias + numComponents);
  float* angularSpeedsConstrained = reinterpret_cast<float*>(linearVelocitiesConstrained + numComponents);
  Vector2* positionsConstrained = reinterpret_cast<Vector2*>(angularSpeedsConstrained + numComponents);
  Rotation* orientationsConstrained = reinterpret_cast<Rotation*>(positionsConstrained + numComponents);
  Vector2* centersOfMassLocal = reinterpret_cast<Vector2*>(orientationsConstrained + numComponents);
  Vector2* centersOfMassWorld = reinterpret_cast<Vector2*>(centersOfMassLocal + numComponents);
  bool* isGravityEnabled = reinterpret_cast<bool*>(centersOfMassWorld + numComponents);
  bool* isInIsland = reinterpret_cast<bool*>(isGravityEnabled + numComponents);
  DynamicArray<uint32>* contactPairs = reinterpret_cast<DynamicArray<uint32>*>(isInIsland + numComponents);

  /* Copy previous data to our new buffers */
  if(mNumComponents) {
    /* Copy component data from the previous buffer to the new buffer */
    memcpy(bodyEntities, mBodyEntities, mNumComponents * sizeof(Entity));
    memcpy(bodies, mBodies, mNumComponents * sizeof(Body*));
    memcpy(colliders, mColliders, mNumComponents * sizeof(DynamicArray<Entity>));
    memcpy(isAllowedToSleep, mIsAllowedToSleep, mNumComponents * sizeof(bool));
    memcpy(isSleeping, mIsSleeping, mNumComponents * sizeof(bool));
    memcpy(sleepTimes, mSleepTimes, mNumComponents * sizeof(float));
    memcpy(types, mTypes, mNumComponents * sizeof(BodyType));
    memcpy(linearVelocities, mLinearVelocities, mNumComponents * sizeof(Vector2));
    memcpy(angularSpeeds, mAngularSpeeds, mNumComponents * sizeof(float));
    memcpy(forces, mForces, mNumComponents * sizeof(Vector2));
    memcpy(torques, mTorques, mNumComponents * sizeof(float));
    memcpy(linearDampings, mLinearDampings, mNumComponents * sizeof(float));
    memcpy(angularDampings, mAngularDampings, mNumComponents * sizeof(float));
    memcpy(masses, mMasses, mNumComponents * sizeof(float));
    memcpy(inverseMasses, mInverseMasses, mNumComponents * sizeof(float));
    memcpy(inertias, mInertias, mNumComponents * sizeof(float));
    memcpy(inverseInertias, mInverseInertias, mNumComponents * sizeof(float));
    memcpy(linearVelocitiesConstrained, mLinearVelocitiesConstrained, mNumComponents * sizeof(Vector2));
    memcpy(angularSpeedsConstrained, mAngularSpeedsConstrained, mNumComponents * sizeof(float));
    memcpy(positionsConstrained, mPositionsConstrained, mNumComponents * sizeof(Vector2));
    memcpy(orientationsConstrained, mOrientationsConstrained, mNumComponents * sizeof(Rotation));
    memcpy(centersOfMassLocal, mCentersOfMassLocal, mNumComponents * sizeof(Vector2));
    memcpy(centersOfMassWorld, mCentersOfMassWorld, mNumComponents * sizeof(Vector2));
    memcpy(isGravityEnabled, mIsGravityEnabled, mNumComponents * sizeof(bool));
    memcpy(isInIsland, mIsInIsland, mNumComponents * sizeof(bool));
    memcpy(contactPairs, mContactPairs, mNumComponents * sizeof(DynamicArray<uint32>));

    /* Release the memory occupied by the previous buffers */
    mMemoryHandler.free(mData, mComponentByteSize * mNumAllocatedComponents);
  }

  /* Assign pointers to component attributes */
  mData = raw;
  mBodyEntities = bodyEntities;
  mBodies = bodies;
  mColliders = colliders;
  mIsAllowedToSleep = isAllowedToSleep;
  mIsSleeping = isSleeping;
  mSleepTimes = sleepTimes;
  mTypes = types;
  mLinearVelocities = linearVelocities;
  mAngularSpeeds = angularSpeeds;
  mForces = forces;
  mTorques = torques;
  mLinearDampings = linearDampings;
  mAngularDampings = angularDampings;
  mMasses = masses;
  mInverseMasses = inverseMasses;
  mInertias = inertias;
  mInverseInertias = inverseInertias;
  mLinearVelocitiesConstrained = linearVelocitiesConstrained;
  mAngularSpeedsConstrained = angularSpeedsConstrained;
  mPositionsConstrained = positionsConstrained;
  mOrientationsConstrained = orientationsConstrained;
  mCentersOfMassLocal = centersOfMassLocal;
  mCentersOfMassWorld = centersOfMassWorld;
  mIsGravityEnabled = isGravityEnabled;
  mIsInIsland = isInIsland;
  mContactPairs = contactPairs;
  mNumAllocatedComponents = numComponents;
}

/* Erase component at the provided index */
void BodyComponents::eraseComponent(uint32 index) {
  Components::eraseComponent(index);
  assert(mEntityComponentMap[mBodyEntities[index]] == index);
  mEntityComponentMap.remove(mBodyEntities[index]);

  /* Destroy individual components */
  mBodyEntities[index].~Entity();
  mBodies[index] = nullptr;
  mColliders[index].~DynamicArray<Entity>();
  mTypes[index].~BodyType();
  mLinearVelocities[index].~Vector2();
  mForces[index].~Vector2();
  mLinearVelocitiesConstrained[index].~Vector2();
  mPositionsConstrained[index].~Vector2();
  mOrientationsConstrained[index].~Rotation();
  mCentersOfMassLocal[index].~Vector2();
  mCentersOfMassWorld[index].~Vector2();
  mContactPairs[index].~DynamicArray<uint32>();
}

/* Move component from one index to another */
void BodyComponents::moveComponent(uint32 source, uint32 destination) {
  const Entity entity = mBodyEntities[source];

  /* Copy source -> destination */
  new (mBodyEntities + destination) Entity(mBodyEntities[source]);
  mBodies[destination] = mBodies[source];
  new (mColliders + destination) DynamicArray<Entity>(mColliders[source]);
  mIsAllowedToSleep[destination] = mIsAllowedToSleep[source];
  mIsSleeping[destination] = mIsSleeping[source];
  mSleepTimes[destination] = mSleepTimes[source];
  mTypes[destination] = mTypes[source];
  new (mLinearVelocities + destination) Vector2(mLinearVelocities[source]);
  mAngularSpeeds[destination] = mAngularSpeeds[source];
  new (mForces + destination) Vector2(mForces[source]);
  mTorques[destination] = mTorques[source];
  mLinearDampings[destination] = mLinearDampings[source];
  mAngularDampings[destination] = mAngularDampings[source];
  mMasses[destination] = mMasses[source];
  mInverseMasses[destination] = mInverseMasses[source];
  mInertias[destination] = mInertias[source];
  mInverseInertias[destination] = mInverseInertias[source];
  new (mLinearVelocitiesConstrained + destination) Vector2(mLinearVelocitiesConstrained[source]);
  mAngularSpeedsConstrained[destination] = mAngularSpeedsConstrained[source];
  new (mPositionsConstrained + destination) Vector2(mPositionsConstrained[source]);
  new (mOrientationsConstrained + destination) Rotation(mOrientationsConstrained[source]);
  new (mCentersOfMassLocal + destination) Vector2(mCentersOfMassLocal[source]);
  new (mCentersOfMassWorld + destination) Vector2(mCentersOfMassWorld[source]);
  mIsGravityEnabled[destination] = mIsGravityEnabled[source];
  mIsInIsland[destination] = mIsInIsland[source];
  new (mContactPairs + destination) DynamicArray<uint32>(mContactPairs[source]);

  /* Destroy source */
  eraseComponent(source);
  assert(!mEntityComponentMap.contains(entity));
  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, destination));
  assert(mEntityComponentMap[mBodyEntities[destination]] == destination);
}

/* Debug */
/* Swap components */
void BodyComponents::swapComponents(uint32 first, uint32 second) {
  /* Copy first component's data */
  Entity firstEntity(mBodyEntities[first]);
  Body* firstBody = mBodies[first];
  DynamicArray<Entity> firstCollider(mColliders[first]);
  bool firstIsAllowedToSleep = mIsAllowedToSleep[first];
  bool firstIsSleeping = mIsSleeping[first];
  float firstSleepTime = mSleepTimes[first];
  BodyType firstBodyType = mTypes[first];
  Vector2 firstLinearVelocity(mLinearVelocities[first]);
  float firstAngularSpeed = mAngularSpeeds[first];
  Vector2 firstExternalForce(mForces[first]);
  float firstExternalTorque = mTorques[first];
  float firstLinearDamping = mLinearDampings[first];
  float firstAngularDamping = mAngularDampings[first];
  float firstMass = mMasses[first];
  float firstInverseMass = mInverseMasses[first];
  float firstInertias = mInertias[first];
  float firstInverseInertias = mInverseInertias[first];
  Vector2 firstConstrainedLinearVelocity(mLinearVelocitiesConstrained[first]);
  float firstConstrainedAngularSpeed = mAngularSpeedsConstrained[first];
  Vector2 firstConstrainedPosition(mPositionsConstrained[first]);
  Rotation firstConstrainedOrientation(mOrientationsConstrained[first]);
  Vector2 firstCenterOfMassLocal(mCentersOfMassLocal[first]);
  Vector2 firstCenterOfMassWorld(mCentersOfMassWorld[first]);
  bool firstIsGravityEnabled = mIsGravityEnabled[first];
  bool firstIsInIsland = mIsInIsland[first];
  DynamicArray<uint32> firstContactPair(mContactPairs[first]);

  /* Destroy first component */
  eraseComponent(first);
  /* Move second's component data to first's location */
  moveComponent(second, first);

  /* Construct first's component data at second */
  new (mBodyEntities + second) Entity(firstEntity);
  mBodies[second] = firstBody;
  new (mColliders + second) DynamicArray<Entity>(firstCollider);
  mIsAllowedToSleep[second] = firstIsAllowedToSleep;
  mIsSleeping[second] = firstIsSleeping;
  mSleepTimes[second] = firstSleepTime;
  mTypes[second] = firstBodyType;
  new (mLinearVelocities + second) Vector2(firstLinearVelocity);
  mAngularSpeeds[second] = firstAngularSpeed;
  new (mForces + second) Vector2(firstExternalForce);
  mTorques[second] = firstExternalTorque;
  mLinearDampings[second] = firstLinearDamping;
  mAngularDampings[second] = firstAngularDamping;
  mMasses[second] = firstMass;
  mInverseMasses[second] = firstInverseMass;
  mInertias[second] = firstInertias;
  mInverseInertias[second] = firstInverseInertias;
  new (mLinearVelocitiesConstrained + second) Vector2(firstConstrainedLinearVelocity);
  mAngularSpeedsConstrained[second] = firstConstrainedAngularSpeed;
  new (mPositionsConstrained + second) Vector2(firstConstrainedPosition);
  new (mOrientationsConstrained + second) Rotation(firstConstrainedOrientation);
  new (mCentersOfMassLocal + second) Vector2(firstCenterOfMassLocal);
  new (mCentersOfMassWorld + second) Vector2(firstCenterOfMassWorld);
  mIsGravityEnabled[second] = firstIsGravityEnabled;
  mIsInIsland[second] = firstIsInIsland;
  new (mContactPairs + second) DynamicArray<uint32>(firstContactPair);

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(firstEntity, second));
  assert(mEntityComponentMap[mBodyEntities[first]] == first);
  assert(mEntityComponentMap[mBodyEntities[second]] == second);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Insert component */
void BodyComponents::insertComponent(Entity entity, bool isSleeping, const BodyComponent& component) {
  /* Find insert index */
  uint32 insertIndex = computeInsertIndex(isSleeping);

  /* Insert component data */
  new (mBodyEntities + insertIndex) Entity(entity);
  mBodies[insertIndex] = component.body;
  new (mColliders + insertIndex) DynamicArray<Entity>(mMemoryHandler);
  mIsAllowedToSleep[insertIndex] = true;
  mIsSleeping[insertIndex] = false;
  mSleepTimes[insertIndex] = 0.0f;
  mTypes[insertIndex] = component.type;
  new (mLinearVelocities + insertIndex) Vector2(0.0f, 0.0f);
  mAngularSpeeds[insertIndex] = 0.0f;
  new (mForces + insertIndex) Vector2(0.0f, 0.0f);
  mTorques[insertIndex] = 0.0f;
  mLinearDampings[insertIndex] = 0.0f;
  mAngularDampings[insertIndex] = 0.0f;
  mMasses[insertIndex] = 1.0f;
  mInverseMasses[insertIndex] = 1.0f;
  mInertias[insertIndex] = 1.0f;
  mInverseInertias[insertIndex] = 1.0f;
  new (mLinearVelocitiesConstrained + insertIndex) Vector2(0.0f, 0.0f);
  mAngularSpeedsConstrained[insertIndex] = 0.0f;
  new (mPositionsConstrained + insertIndex) Vector2(0.0f, 0.0);
  new (mOrientationsConstrained + insertIndex) Rotation(0.0f, 1.0f);
  new (mCentersOfMassLocal + insertIndex) Vector2(0.0f, 0.0f);
  new (mCentersOfMassWorld + insertIndex) Vector2(component.worldPosition);
  mIsGravityEnabled[insertIndex] = true;
  mIsInIsland[insertIndex] = false;
  new (mContactPairs + insertIndex) DynamicArray<uint32>(mMemoryHandler);

  /* Update entity-component map */
  mEntityComponentMap.insert(Pair<Entity, uint32>(entity, insertIndex));
  mNumComponents++;
  assert(mSleepingStartIndex <= mNumComponents);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Get pointer to body */
Body* BodyComponents::getBody(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mBodies[mEntityComponentMap[entity]];
}

/* Add a collider to the body */
void BodyComponents::addCollider(Entity bodyEntity, Entity colliderEntity) {
  assert(mEntityComponentMap.contains(bodyEntity));
  mColliders[mEntityComponentMap[bodyEntity]].add(colliderEntity);
}

/* Remove a collider from the body */
void BodyComponents::removeCollider(Entity bodyEntity, Entity colliderEntity) {
  assert(mEntityComponentMap.contains(bodyEntity));
  mColliders[mEntityComponentMap[bodyEntity]].remove(colliderEntity);
}

/* Get the colliders of the body */
const DynamicArray<Entity>& BodyComponents::getColliders(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mColliders[mEntityComponentMap[entity]];
}

/* Get sleep permission */
bool BodyComponents::getIsAllowedToSleep(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mIsAllowedToSleep[mEntityComponentMap[entity]];
}

/* Set sleep permission */
void BodyComponents::setIsAllowedToSleep(Entity entity, bool isAllowedToSleep) {
    assert(mEntityComponentMap.contains(entity));
    mIsAllowedToSleep[mEntityComponentMap[entity]] = isAllowedToSleep;
}

/* Query sleep status */
bool BodyComponents::getIsSleeping(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mIsSleeping[mEntityComponentMap[entity]];
}

/* Set sleep status */
void BodyComponents::setIsSleeping(Entity entity, bool isSleeping) {
    assert(mEntityComponentMap.contains(entity));
    mIsSleeping[mEntityComponentMap[entity]] = isSleeping;
}

/* Get sleep time */
float BodyComponents::getSleepTime(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mSleepTimes[mEntityComponentMap[entity]];
}

/* Set sleep time */
void BodyComponents::setSleepTime(Entity entity, float sleepTime) {
    assert(mEntityComponentMap.contains(entity));
    mSleepTimes[mEntityComponentMap[entity]] = sleepTime;
}

/* Get body type */
BodyType BodyComponents::getType(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mTypes[mEntityComponentMap[entity]];
}

/* Set body type */
void BodyComponents::setType(Entity entity, BodyType type) {
    assert(mEntityComponentMap.contains(entity));
    mTypes[mEntityComponentMap[entity]] = type;
}

/* Get linear velocity */
const Vector2& BodyComponents::getLinearVelocity(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mLinearVelocities[mEntityComponentMap[entity]];
}

/* Set linear velocity */
void BodyComponents::setLinearVelocity(Entity entity, const Vector2& linearVelocity) {
    assert(mEntityComponentMap.contains(entity));
    mLinearVelocities[mEntityComponentMap[entity]] = linearVelocity;
}

/* Get angular speed */
float BodyComponents::getAngularSpeed(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mAngularSpeeds[mEntityComponentMap[entity]];
}

/* Set angular speed */
void BodyComponents::setAngularSpeed(Entity entity, float angularSpeed) {
    assert(mEntityComponentMap.contains(entity));
    mAngularSpeeds[mEntityComponentMap[entity]] = angularSpeed;
}

/* Get external force */
const Vector2& BodyComponents::getForce(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mForces[mEntityComponentMap[entity]];
}

/* Set external force */
void BodyComponents::setForce(Entity entity, const Vector2& force) {
    assert(mEntityComponentMap.contains(entity));
    mForces[mEntityComponentMap[entity]] = force;
}

/* Get external torque */
float BodyComponents::getTorque(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mTorques[mEntityComponentMap[entity]];
}

/* Set external torque */
void BodyComponents::setTorque(Entity entity, float torque) {
    assert(mEntityComponentMap.contains(entity));
    mTorques[mEntityComponentMap[entity]] = torque;
}

/* Get linear damping */
float BodyComponents::getLinearDamping(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mLinearDampings[mEntityComponentMap[entity]];
}

/* Set linear damping */
void BodyComponents::setLinearDamping(Entity entity, float linearDamping) {
    assert(mEntityComponentMap.contains(entity));
    mLinearDampings[mEntityComponentMap[entity]] = linearDamping;
}

/* Get angular damping */
float BodyComponents::getAngularDamping(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mAngularDampings[mEntityComponentMap[entity]];
}

/* Set angular damping */
void BodyComponents::setAngularDamping(Entity entity, float angularDamping) {
    assert(mEntityComponentMap.contains(entity));
    mAngularDampings[mEntityComponentMap[entity]] = angularDamping;
}

/* Get mass */
float BodyComponents::getMass(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mMasses[mEntityComponentMap[entity]];
}

/* Set mass */
void BodyComponents::setMass(Entity entity, float mass) {
    assert(mEntityComponentMap.contains(entity));
    mMasses[mEntityComponentMap[entity]] = mass;
}

/* Get inverse mass */
float BodyComponents::getInverseMass(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mInverseMasses[mEntityComponentMap[entity]];
}

/* Set inverse mass */
void BodyComponents::setInverseMass(Entity entity, float inverseMass) {
    assert(mEntityComponentMap.contains(entity));
    mInverseMasses[mEntityComponentMap[entity]] = inverseMass;
}

/* Get rotational inertia */
float BodyComponents::getInertia(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mInertias[mEntityComponentMap[entity]];
}

/* Set rotational inertia */
void BodyComponents::setInertia(Entity entity, float inertia) {
    assert(mEntityComponentMap.contains(entity));
    mInertias[mEntityComponentMap[entity]] = inertia;
}

/* Get inverse rotational inertia */
float BodyComponents::getInverseInertia(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mInverseInertias[mEntityComponentMap[entity]];
}

/* Set inverse rotational inertia */
void BodyComponents::setInverseInertia(Entity entity, float inverseInertia) {
    assert(mEntityComponentMap.contains(entity));
    mInverseInertias[mEntityComponentMap[entity]] = inverseInertia;
}

/* Set constrained linear velocity */
void BodyComponents::setConstrainedLinearVelocity(Entity entity, const Vector2& constrainedLinearVelocity) {
    assert(mEntityComponentMap.contains(entity));
    mLinearVelocitiesConstrained[mEntityComponentMap[entity]] = constrainedLinearVelocity;
}

/* Set constrained angular speed */
void BodyComponents::setConstrainedAngularSpeed(Entity entity, float constrainedAngularSpeed) {
    assert(mEntityComponentMap.contains(entity));
    mAngularSpeedsConstrained[mEntityComponentMap[entity]] = constrainedAngularSpeed;
}

/* Get constrained position */
Vector2& BodyComponents::getConstrainedPosition(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mPositionsConstrained[mEntityComponentMap[entity]];
}

/* Set constrained position */
void BodyComponents::setConstrainedPosition(Entity entity, const Vector2& constrainedPosition) {
    assert(mEntityComponentMap.contains(entity));
    mPositionsConstrained[mEntityComponentMap[entity]] = constrainedPosition;
}

/* Get constrained orientation */
Rotation& BodyComponents::getConstrainedOrientation(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mOrientationsConstrained[mEntityComponentMap[entity]];
}

/* Set constrained orientation */
void BodyComponents::setConstrainedOrientation(Entity entity, const Rotation& constrainedOrientation) {
    assert(mEntityComponentMap.contains(entity));
    mOrientationsConstrained[mEntityComponentMap[entity]] = constrainedOrientation;
}

/* Get local center of mass */
const Vector2& BodyComponents::getCenterOfMassLocal(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mCentersOfMassLocal[mEntityComponentMap[entity]];
}

/* Set local center of mass */
void BodyComponents::setCenterOfMassLocal(Entity entity, const Vector2& centerOfMassLocal) {
    assert(mEntityComponentMap.contains(entity));
    mCentersOfMassLocal[mEntityComponentMap[entity]] = centerOfMassLocal;
}

/* Get world center of mass */
const Vector2& BodyComponents::getCenterOfMassWorld(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mCentersOfMassWorld[mEntityComponentMap[entity]];
}

/* Set world center of mass */
void BodyComponents::setCenterOfMassWorld(Entity entity, const Vector2& centerOfMassWorld) {
    assert(mEntityComponentMap.contains(entity));
    mCentersOfMassWorld[mEntityComponentMap[entity]] = centerOfMassWorld;
}

/* Get gravitational state */
bool BodyComponents::getIsGravityEnabled(Entity entity) const {
    assert(mEntityComponentMap.contains(entity));
    return mIsGravityEnabled[mEntityComponentMap[entity]];
}

/* Set gravitational state */
void BodyComponents::setIsGravityEnabled(Entity entity, bool isGravityEnabled) {
    assert(mEntityComponentMap.contains(entity));
    mIsGravityEnabled[mEntityComponentMap[entity]] = isGravityEnabled;
}

/* Get island inclusion state */
bool BodyComponents::getIsInIsland(Entity entity) const {
  assert(mEntityComponentMap.contains(entity));
  return mIsInIsland[mEntityComponentMap[entity]];
}

/* Set island inclusion state */
void BodyComponents::setIsInIsland(Entity entity, bool isInIsland) {
  assert(mEntityComponentMap.contains(entity));
  mIsInIsland[mEntityComponentMap[entity]] = isInIsland;
}

/* Add contact pairing index */
void BodyComponents::addContactPair(Entity entity, uint32 index) {
    assert(mEntityComponentMap.contains(entity));
    mContactPairs[mEntityComponentMap[entity]].add(index);
}

