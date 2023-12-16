#include <physics/common/World.h>
#include <physics/dynamics/Body.h>
#include <physics/collision/Shape.h>
#include <physics/common/Factory.h>

using namespace physics;

/* Constructor */
Body::Body(World& world, Entity entity) : mEntity(entity), mWorld(world) {}

/* Remove all of the overlapping pairs that this body is involved in */
void Body::resetOverlapPairs() {
  /* Colliders associated with the current body */
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    /* Overlap pairs associated with the current collider of the current body */
    DynamicArray<uint64> overlapPairs = mWorld.mColliderComponents.getOverlapPairs(colliderEntities[i]);
    const uint64 numOverlapPairs = overlapPairs.size();

    for(uint64 j = 0; j < numOverlapPairs; j++) {
      mWorld.mCollisionDetection.mOverlapPairs.eraseOverlapPair(overlapPairs[j]);
    }
  }

  /* Recompute overlap pairs in the next frame */
  checkBroadPhaseCollision();
}

/* Remove all collision shapes */
void Body::removeColliders() {
  const DynamicArray<Entity> colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  /* Obtain the pointer for the collider entity an then remove it from the components array  */
  for(uint32 i = 0; i < numColliderEntities; i++) {
    removeCollider(mWorld.mColliderComponents.getCollider(colliderEntities[i]));
  }
}

/* Update state in broad phase */
void Body::updateBroadPhase() const {
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    /* Update local to world transform of the collider as well as its broad phase state */
    mWorld.mColliderComponents.setTransformLocalWorld(colliderEntities[i], mWorld.mTransformComponents.getTransform(mEntity) * mWorld.mColliderComponents.getTransformLocalBody(colliderEntities[i]));
    mWorld.mCollisionDetection.updateCollider(colliderEntities[i]);
  }
}

/* Check the collision shapes of the body for collision in broadphase */
void Body::checkBroadPhaseCollision() const {
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    Collider* collider = mWorld.mColliderComponents.getCollider(colliderEntities[i]);
    /* Ask broad phase to check collision through the dynamic tree */
    mWorld.mCollisionDetection.checkBroadPhaseCollision(collider);
  }
}

/* Get the entity of the body */
Entity Body::getEntity() const {
  return mEntity;
}

/* Create a new collider and add it to the body */
Collider* Body::addCollider(Shape* shape, const Transform& transform) {
  /* New entity for the collider */
  Entity colliderEntity = mWorld.mEntityHandler.createEntity();
  /* Create the actual collider */
  Collider* collider = new (mWorld.mMemoryStrategy.allocate(MemoryStrategy::HandlerType::ObjectPool, sizeof(Collider))) Collider(colliderEntity, this, mWorld.mMemoryStrategy);
  Vector2 lowerBound;
  Vector2 upperBound;
  /* Compute relevant quantities and add the collider to the components array */
  shape->getLocalBounds(lowerBound, upperBound);
  const Transform transformLocalWorld = mWorld.mTransformComponents.getTransform(mEntity) * transform;
  Material material(mWorld.mSettings.defaultFrictionConstant, mWorld.mSettings.defaultRestitutionConstant);
  ColliderComponents::ColliderComponent colliderComponent(mEntity, collider, AABB(lowerBound, upperBound), transform, transformLocalWorld, material, shape, 0x0001, 0xFFFF);
  bool isSleeping = mWorld.mBodyComponents.getIsSleeping(mEntity);
  mWorld.mColliderComponents.insertComponent(colliderEntity, isSleeping, colliderComponent);
  mWorld.mBodyComponents.addCollider(mEntity, colliderEntity);
  /* Associate the collider with the provided collision shape */
  shape->addCollider(collider);
  AABB aabb;
  shape->computeAABB(aabb, mWorld.mTransformComponents.getTransform(mEntity) * transform);
  /* Add the collider into broad phase */
  mWorld.mCollisionDetection.addCollider(collider, aabb);
  LOG("Added collider index " + std::to_string(colliderEntity.getIndex()) + " to body index " + std::to_string(mEntity.getIndex()));
  return collider;
}

/* Remove a collider from the body */
void Body::removeCollider(Collider* collider) {
  LOG("Removing collider index " + std::to_string(collider->getEntity().getIndex()) + " from body index " + std::to_string(mEntity.getIndex()));
  /* Remove the collider from broad phase */
  if(collider->getBroadPhaseIdentifier() != -1) {
    mWorld.mCollisionDetection.removeCollider(collider);
  }

  mWorld.mBodyComponents.removeCollider(mEntity, collider->getEntity());
  /* Remove the associated of the collider with its collision shape */
  collider->getShape()->removeCollider(collider);
  /* Remove the collider from the components array */
  mWorld.mColliderComponents.removeComponent(collider->getEntity());
  /* Destroy the collider's entity */
  mWorld.mEntityHandler.deleteEntity(collider->getEntity());
  /* Destroy the actual collider */
  collider->~Collider();
  /* Release memory that was occupied by the collider */
  mWorld.mMemoryStrategy.free(MemoryStrategy::HandlerType::ObjectPool, collider, sizeof(Collider));
}

/* Get a constant pointer to a given collider of the body */
const Collider* Body::getCollider(uint32 index) const {
  assert(index < getNumColliders());
  Entity colliderEntity = mWorld.mBodyComponents.getColliders(mEntity)[index];
  return mWorld.mColliderComponents.getCollider(colliderEntity);
}

Collider* Body::getCollider(uint32 index) {
  assert(index < getNumColliders());
  Entity colliderEntity = mWorld.mBodyComponents.getColliders(mEntity)[index];
  return mWorld.mColliderComponents.getCollider(colliderEntity);
}

/* Get number of colliders */
uint32 Body::getNumColliders() const {
  return static_cast<uint32>(mWorld.mBodyComponents.getColliders(mEntity).size());
}

/* Query whether a point is inside the body */
bool Body::testPoint(const Vector2& pointWorld) const {
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    Collider* collider = mWorld.mColliderComponents.getCollider(colliderEntities[i]);

    /* Test for presence of point inside the collider */
    if(collider->testPoint(pointWorld)) {
      return true;
    }
  }

  return false;
}

/* Query whether the body overlaps with the given AABB */
bool Body::testOverlap(const AABB& aabb) const {
  return aabb.isOverlapping(getAABB());
}

/* Get the body's AABB by merging all of its colliders' AABBs */
AABB Body::getAABB() const {
  AABB aabb;
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  if(!numColliderEntities) {
    return aabb;
  }

  const Transform& transform = mWorld.mTransformComponents.getTransform(mEntity);
  Collider* collider = mWorld.mColliderComponents.getCollider(colliderEntities[0]);
  collider->getShape()->computeAABB(aabb, transform * collider->getTransformLocalBody());

  for(uint32 i = 1; i < numColliderEntities; i++) {
    Collider* collider = mWorld.mColliderComponents.getCollider(colliderEntities[i]);
    AABB aabbToMerge;
    /* World space AABB of current collider */
    collider->getShape()->computeAABB(aabbToMerge, transform * collider->getTransformLocalBody());
    /* Merge bounds of current collider AABB with the body AABB so far */
    aabb.combine(aabbToMerge);
  }

  return aabb;
}

/* Get current position and orientation */
const Transform& Body::getTransform() const {
  return mWorld.mTransformComponents.getTransform(mEntity);
}

/* Set current position and orientation */
void Body::setTransform(const Transform& transform) {
  const Vector2 oldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);
  /* Center of mass in local space coordinates */
  const Vector2& localCenterOfMass = mWorld.mBodyComponents.getCenterOfMassLocal(mEntity);
  /* Center of mass in world space coordinates */
  mWorld.mBodyComponents.setCenterOfMassWorld(mEntity, transform * localCenterOfMass);
  /* Update the linear velocity of the center of mass */
  Vector2 linearVelocity = mWorld.mBodyComponents.getLinearVelocity(mEntity);
  const float angularSpeed = mWorld.mBodyComponents.getAngularSpeed(mEntity);
  const Vector2& worldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);
  const Vector2 displacement = worldCenterOfMass - oldCenterOfMass;
  linearVelocity += cross(angularSpeed, displacement);
  mWorld.mBodyComponents.setLinearVelocity(mEntity, linearVelocity);
  mWorld.mTransformComponents.setTransform(mEntity, transform);
  /* Update the broad phase state */
  updateBroadPhase();
  /* Awake the body now that it has been forcibly move */
  setIsSleeping(false);
}

/* Get mass of the body */
float Body::getMass() const {
  return mWorld.mBodyComponents.getMass(mEntity);
}

/* Set mass of the body */
void Body::setMass(float mass) {
  /* Cannot assign negative mass */
  if(mass < 0.0f) {
    return;
  }

  mWorld.mBodyComponents.setMass(mEntity, mass);
  const BodyType type = mWorld.mBodyComponents.getType(mEntity);

  /* Dynamic bodies are the only body types where inverse mass is necessary */
  if(type == BodyType::Dynamic) {
    mWorld.mBodyComponents.setInverseMass(mEntity, mass > 0.0f ? 1.0f / mass : 0.0f);
  }
}

/* Get linear velocity of the body */
Vector2 Body::getLinearVelocity() const {
  return mWorld.mBodyComponents.getLinearVelocity(mEntity);
}

/* Set linear velocity of the body */
void Body::setLinearVelocity(const Vector2& linearVelocity) {
  /* Static bodies do no have velocity */
  if(mWorld.mBodyComponents.getType(mEntity) == BodyType::Static) {
    return;
  }

  mWorld.mBodyComponents.setLinearVelocity(mEntity, linearVelocity);

  /* Awake the body now that its linear velocity has been set */
  if(linearVelocity.lengthSquare() > 0.0f) {
    setIsSleeping(false);
  }
}

/* Get angular speed of the body */
float Body::getAngularSpeed() const {
  return mWorld.mBodyComponents.getAngularSpeed(mEntity);
}

/* Set angular speed of the body */
void Body::setAngularSpeed(float angularSpeed) {
  /* Static bodies do no have angular speed */
  if(mWorld.mBodyComponents.getType(mEntity) == BodyType::Static) {
    return;
  }

  mWorld.mBodyComponents.setAngularSpeed(mEntity, angularSpeed);

  /* Awake the body now that its angular speed has been set */
  if(square(angularSpeed) > 0.0f) {
    setIsSleeping(false);
  }
}

/* Get linear damping of the body */
float Body::getLinearDamping() const {
  return mWorld.mBodyComponents.getLinearDamping(mEntity);
}

/* Set linear damping of the body */
void Body::setLinearDamping(float linearDamping) {
  assert(linearDamping >= 0.0f);

  if(linearDamping >= 0.0f) {
    mWorld.mBodyComponents.setLinearDamping(mEntity, linearDamping);
  }
}

/* Get angular damping of the body */
float Body::getAngularDamping() const {
  return mWorld.mBodyComponents.getAngularDamping(mEntity);
}

/* Set angular damping of the body */
void Body::setAngularDamping(float angularDamping) {
  assert(angularDamping >= 0.0f);

  if(angularDamping >= 0.0f) {
    mWorld.mBodyComponents.setAngularDamping(mEntity, angularDamping);
  }
}

/* Get inertia of the body */
float Body::getInertia() const {
  return mWorld.mBodyComponents.getInertia(mEntity);
}

/* Set inertia of the body */
void Body::setInertia(float inertia) {
  /* Inertia is only necessary for dynamic bodies */
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  assert(inertia > 0.0f);

  if(inertia > 0.0f) {
    float mass = mWorld.mBodyComponents.getMass(mEntity);
    /* Debug */
    Vector2 center = mWorld.mBodyComponents.getCenterOfMassLocal(mEntity);
    /* Center the inertia about the center of mass  */
    inertia = inertia - (mass * dot(center, center));
    mWorld.mBodyComponents.setInertia(mEntity, inertia);
    mWorld.mBodyComponents.setInverseInertia(mEntity, 1.0f / inertia);
  }
  else {
    mWorld.mBodyComponents.setInertia(mEntity, 0.0f);
    mWorld.mBodyComponents.setInverseInertia(mEntity, 0.0f);
  }
}

/* Get local center of mass */
const Vector2& Body::getCenterOfMassLocal() const {
  return mWorld.mBodyComponents.getCenterOfMassLocal(mEntity);
}

/* Set local center of mass */
void Body::setCenterOfMassLocal(const Vector2& centerOfMass) {
  const Vector2 oldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);
  /* Center of mass in local space coordinates */
  mWorld.mBodyComponents.setCenterOfMassLocal(mEntity, centerOfMass);
  /* Center of mass in world space coordinates */
  mWorld.mBodyComponents.setCenterOfMassWorld(mEntity, mWorld.mTransformComponents.getTransform(mEntity) * centerOfMass);
  /* Update the linear velocity of the center of mass */
  Vector2 linearVelocity = mWorld.mBodyComponents.getLinearVelocity(mEntity);
  const float angularSpeed = mWorld.mBodyComponents.getAngularSpeed(mEntity);
  const Vector2& worldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);
  const Vector2 displacement = worldCenterOfMass - oldCenterOfMass;
  linearVelocity += cross(angularSpeed, displacement);
  mWorld.mBodyComponents.setLinearVelocity(mEntity, linearVelocity);
}

/* Set all mass properties of the body using the body's colliders */
void Body::setMassPropertiesUsingColliders() {
  float mass = 0.0f;
  float inverseMass = 0.0f;
  float inertia = 0.0f;
  float inverseInertia = 0.0f;
  Vector2 localCenterOfMass(0.0f, 0.0f);
  Vector2 worldCenterOfMass(0.0f, 0.0f);
  const Vector2& oldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);

  /* Static and kinematic bodies do not have any mass */
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    mWorld.mBodyComponents.setCenterOfMassWorld(mEntity, mWorld.mTransformComponents.getTransform(mEntity).getPosition());
    mWorld.mBodyComponents.setCenterOfMassLocal(mEntity, localCenterOfMass);
    mWorld.mBodyComponents.setInertia(mEntity, inertia);
    mWorld.mBodyComponents.setInverseInertia(mEntity, inverseInertia);
    mWorld.mBodyComponents.setMass(mEntity, mass);
    mWorld.mBodyComponents.setInverseMass(mEntity, inverseMass);
    return;
  }

  assert(mWorld.mBodyComponents.getType(mEntity) == BodyType::Dynamic);

  /* Collect all colliders of the current body */
  const DynamicArray<Entity>& colliderEntities = mWorld.mBodyComponents.getColliders(mEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  /* For each collider, accumulate area, center of mass, mass and inertia */
  for(uint32 i = 0; i < numColliderEntities; i++) {
    const uint32 colliderIndex = mWorld.mColliderComponents.getComponentEntityIndex(colliderEntities[i]);
    const float colliderDensity = mWorld.mColliderComponents.mMaterials[colliderIndex].getDensity();

    if(colliderDensity == 0.0f) {
      continue;
    }

    const float colliderArea = mWorld.mColliderComponents.mShapes[colliderIndex]->getArea();
    const Vector2& colliderCentroid = mWorld.mColliderComponents.mShapes[colliderIndex]->getCentroid();
    const float colliderMass = colliderArea * colliderDensity;
    mass += colliderMass;
    localCenterOfMass += (colliderMass * colliderCentroid);
    inertia += mWorld.mColliderComponents.mShapes[colliderIndex]->getLocalInertia(colliderMass);
  }

  /* Compute local center of mass */
  if(mass > 0.0f) {
    inverseMass = 1.0f / mass;
    localCenterOfMass *= inverseMass;
  }

  if(inertia > 0.0f) {
    /* Center inertia about the local center of mass */
    inertia -= (mass * dot(localCenterOfMass, localCenterOfMass));
    assert(inertia > 0.0f);
    inverseInertia = 1.0f / inertia;
  }
  else {
    inertia = 0.0f;
    inverseInertia = 0.0f;
  }

  /* Move the center of mass and assign mass properties to the body's component */
  worldCenterOfMass = mWorld.mTransformComponents.getTransform(mEntity) * localCenterOfMass;
  mWorld.mBodyComponents.setCenterOfMassWorld(mEntity, worldCenterOfMass);
  mWorld.mBodyComponents.setCenterOfMassLocal(mEntity, localCenterOfMass);
  mWorld.mBodyComponents.setInertia(mEntity, inertia);
  mWorld.mBodyComponents.setInverseInertia(mEntity, inverseInertia);
  mWorld.mBodyComponents.setMass(mEntity, mass);
  mWorld.mBodyComponents.setInverseMass(mEntity, inverseMass);

  /* Update the center of mass velocity */
  Vector2 linearVelocity = mWorld.mBodyComponents.getLinearVelocity(mEntity);
  const float angularSpeed = mWorld.mBodyComponents.getAngularSpeed(mEntity);
  const Vector2 displacement = worldCenterOfMass - oldCenterOfMass;
  linearVelocity += cross(angularSpeed, displacement);
  mWorld.mBodyComponents.setLinearVelocity(mEntity, linearVelocity);
}

/* Get type of the body */
BodyType Body::getType() const {
  return mWorld.mBodyComponents.getType(mEntity);
}

/* Set type of the body */
void Body::setType(BodyType type) {
  if(mWorld.mBodyComponents.getType(mEntity) == type) {
    return;
  }

  mWorld.mBodyComponents.setType(mEntity, type);

  if(type == BodyType::Static) {
    /* Static bodies have no velocity */
    mWorld.mBodyComponents.setLinearVelocity(mEntity, Vector2::getZeroVector());
    mWorld.mBodyComponents.setAngularSpeed(mEntity, 0.0f);
  }

  if(type == BodyType::Static || type == BodyType::Kinematic) {
    /* Disregard inverse mass and inertia for static and kinematic bodies */
    mWorld.mBodyComponents.setInverseMass(mEntity, 0.0f);
    mWorld.mBodyComponents.setInverseInertia(mEntity, 0.0f);
  }
  else {
    /* Body is dynamic */
    const float mass = mWorld.mBodyComponents.getMass(mEntity);
    const float inertia = mWorld.mBodyComponents.getInertia(mEntity);
    mWorld.mBodyComponents.setInverseMass(mEntity, mass > 0.0f ? 1.0f / mass : 0.0f);
    mWorld.mBodyComponents.setInverseInertia(mEntity, inertia > 0.0f ? 1.0f / inertia : 0.0f);
  }

  /* Reset stimuli */
  mWorld.mBodyComponents.setForce(mEntity, Vector2::getZeroVector());
  mWorld.mBodyComponents.setTorque(mEntity, 0.0f);
  setIsSleeping(false);
  resetOverlapPairs();
}

/* Query whether gravity is enabled for this body */
bool Body::isGravityEnabled() const {
  return mWorld.mBodyComponents.getIsGravityEnabled(mEntity);
}

/* Set whether gravity is enabled for this body */
void Body::setIsGravityEnabled(bool isGravityEnabled) {
  mWorld.mBodyComponents.setIsGravityEnabled(mEntity, isGravityEnabled);
}

/* Query whether this body is allowed to sleep */
bool Body::isAllowedToSleep() const {
  return mWorld.mBodyComponents.getIsAllowedToSleep(mEntity);
}

/* Set whether this body is allowed to sleep */
void Body::setIsAllowedToSleep(bool isAllowedToSleep) {
  mWorld.mBodyComponents.setIsAllowedToSleep(mEntity, isAllowedToSleep);
}

/* Query whether the body is sleeping */
bool Body::isSleeping() const {
  return mWorld.mBodyComponents.getIsSleeping(mEntity);
}

/* Set whether the body is sleeping */
void Body::setIsSleeping(bool isSleeping) {
  mWorld.mBodyComponents.setIsSleeping(mEntity, isSleeping);
}

/* Apply world force to body at world point */
void Body::applyForce(const Vector2& force, const Vector2& point) {
  /* Forces can only be applied to dynamic bodies */
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  /* Awake the body now that we have applied a force externally */
  if(mWorld.mBodyComponents.getIsSleeping(mEntity)) {
    setIsSleeping(false);
  }

  const Vector2& totalForce = mWorld.mBodyComponents.getForce(mEntity);
  const Vector2& worldCenterOfMass = mWorld.mBodyComponents.getCenterOfMassWorld(mEntity);\
  /* Forces applied to bodies outside their center of mass produce a torque so we need to account for this */
  const float totalTorque = mWorld.mBodyComponents.getTorque(mEntity);
  mWorld.mBodyComponents.setForce(mEntity, totalForce + force);
  mWorld.mBodyComponents.setTorque(mEntity, totalTorque + cross(point - worldCenterOfMass, force));
}

/* Apply world force to body at world center of mass position */
void Body::applyForceToCenter(const Vector2& force) {
  /* Forces can only be applied to dynamic bodies */
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  /* Awake the body now that we have applied a force externally */
  if(mWorld.mBodyComponents.getIsSleeping(mEntity)) {
    setIsSleeping(false);
  }

  const Vector2& totalForce = mWorld.mBodyComponents.getForce(mEntity);
  mWorld.mBodyComponents.setForce(mEntity, totalForce + force);
}

/* Apply world torque to body */
void Body::applyTorque(float torque) {
  /* Torques can only be applied to dynamic bodies */
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  /* Awake the body now that we have applied a torque externally */
  if(mWorld.mBodyComponents.getIsSleeping(mEntity)) {
    setIsSleeping(false);
  }

  const float totalTorque = mWorld.mBodyComponents.getTorque(mEntity);
  mWorld.mBodyComponents.setTorque(mEntity, totalTorque + torque);
}

/* Clear the total force acting on the body */
void Body::clearForces() {
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  mWorld.mBodyComponents.setForce(mEntity, Vector2::getZeroVector());
}

/* Clear the total torque acting on the body */
void Body::clearTorques() {
  if(mWorld.mBodyComponents.getType(mEntity) != BodyType::Dynamic) {
    return;
  }

  mWorld.mBodyComponents.setTorque(mEntity, 0.0f);
}

/* Get the total force acting on the body */
const Vector2& Body::getForce() const {
  return mWorld.mBodyComponents.getForce(mEntity);
}

/* Get the total torque acting on the body */
float Body::getTorque() const {
  return mWorld.mBodyComponents.getTorque(mEntity);
}