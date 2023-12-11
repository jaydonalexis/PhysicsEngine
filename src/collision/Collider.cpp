#include <physics/collision/Collider.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/common/World.h>

using namespace physics;

/* Constructor */
Collider::Collider(Entity entity,
                   Body* body,
                   MemoryStrategy& memoryStrategy) :
                   mEntity(entity),
                   mBody(body),
                   mMemoryStrategy(memoryStrategy) {}

/* Collider's representative collision shape has been changed */
void Collider::setShapeSizeChanged(bool shapeSizeChanged) {
  mBody->mWorld.mColliderComponents.setHasSizeChanged(mEntity, shapeSizeChanged);
}

/* Get collider's identifier */
Entity Collider::getEntity() const {
  return mEntity;
}

/* Return a pointer to the collider's representative collision shape */
Shape* Collider::getShape() {
  return mBody->mWorld.mColliderComponents.getShape(mEntity);
}

/* Return a const pointer to the collider's representative collision shape */
const Shape* Collider::getShape() const {
  return mBody->mWorld.mColliderComponents.getShape(mEntity);
}

/* Return a pointer to the collider's owning body */
Body* Collider::getBody() const {
  return mBody;
}

/* Get the local to owning body transform */
const Transform& Collider::getTransformLocalBody() const {
  return mBody->mWorld.mColliderComponents.getTransformLocalBody(mEntity);
}

/* Set the local to owning body transform */
void Collider::setTransformLocalBody(const Transform& transform) {
  mBody->mWorld.mColliderComponents.setTransformLocalBody(mEntity, transform);
  /* Need to synchronize the local to world transform as well */
  const Transform& bodyTransform = mBody->mWorld.mTransformComponents.getTransform(mBody->getEntity());
  mBody->mWorld.mColliderComponents.setTransformLocalWorld(mEntity, bodyTransform * transform);
  Body* body = static_cast<Body*>(mBody);

  if(body) {
    body->setIsSleeping(false);
  }

  /* Update the broad phase state of the collider */
  mBody->mWorld.mCollisionDetection.updateCollider(mEntity);
}

/* Get the local to world transform */
const Transform& Collider::getTransformLocalWorld() const {
  return mBody->mWorld.mColliderComponents.getTransformLocalWorld(mEntity);
}

/* Debug */
/* Return world space AABB of the collider */
const AABB Collider::getAABB() const {
  AABB aabb;
  Shape* shape = mBody->mWorld.mColliderComponents.getShape(mEntity);
  shape->computeAABB(aabb, getTransformLocalWorld());
  return aabb;
}

/* Query whether the collider overlaps with the given AABB */
bool Collider::testOverlap(const AABB& aabb) {
  return aabb.isOverlapping(getAABB());
}

/* Debug */
/* Query whether a point is inside the collider's representative collision shape */
bool Collider::testPoint(const Vector2& point) {
  const Transform transformLocalWorld = mBody->mWorld.mTransformComponents.getTransform(mBody->getEntity()) *
                                        mBody->mWorld.mColliderComponents.getTransformLocalBody(mEntity);
  /* Debug */
  const Vector2 pointLocal = transformLocalWorld ^ point;
  const Shape* shape = mBody->mWorld.mColliderComponents.getShape(mEntity);
  return shape->testPoint(pointLocal);
}

/* Get collision category */
unsigned short Collider::getCollisionCategory() const {
  return mBody->mWorld.mColliderComponents.getCollisionCategory(mEntity);
}

/* Set collision category */
void Collider::setCollisionCategory(unsigned short collisionCategory) {
  mBody->mWorld.mColliderComponents.setCollisionCategory(mEntity, collisionCategory);
  /* Check for a broad phase collision in the next frame now that we have upated the collision category */
  mBody->mWorld.mCollisionDetection.checkBroadPhaseCollision(this);
}

/* Get Collision filter */
unsigned short Collider::getCollisionFilter() const {
  return mBody->mWorld.mColliderComponents.getCollisionFilter(mEntity);
}

/* Set Collision filter */
void Collider::setCollisionFilter(unsigned short collisionFilter) {
  mBody->mWorld.mColliderComponents.setCollisionFilter(mEntity, collisionFilter);
  /* Check for a broad phase collision in the next frame now that we have updated the collision filter */
  mBody->mWorld.mCollisionDetection.checkBroadPhaseCollision(this);
}

/* Get broad phase identifier */
int32 Collider::getBroadPhaseIdentifier() const {
  return mBody->mWorld.mColliderComponents.getBroadPhaseIdentifier(mEntity);
}

/* Return a reference to the collider's material */
Material& Collider::getMaterial() {
  return mBody->mWorld.mColliderComponents.getMaterial(mEntity);
}

/* Set the collider's material */
void Collider::setMaterial(const Material& material) {
  mBody->mWorld.mColliderComponents.setMaterial(mEntity, material);
}