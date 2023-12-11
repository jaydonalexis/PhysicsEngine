#include <physics/collision/BroadPhase.h>
#include <physics/memory/MemoryStrategy.h>
#include <physics/common/World.h>

using namespace physics;

/* Constructor */
BroadPhase::BroadPhase(CollisionDetection& collisionDetection,
                       BodyComponents& bodyComponents,
                       ColliderComponents& colliderComponents,
                       TransformComponents& transformComponents) :
                       mDynamicTree(collisionDetection.getMemoryStrategy().getFreeListMemoryHandler(), DYNAMIC_TREE_FAT_AABB_INFLATION),
                       mBodyComponents(bodyComponents),
                       mColliderComponents(colliderComponents),
                       mTransformComponents(transformComponents),
                       mShapesToTest(collisionDetection.getMemoryStrategy().getFreeListMemoryHandler()),
                       mCollisionDetection(collisionDetection) {}

/* Notify tree about collider update */
void BroadPhase::notifyColliderUpdate(int32 broadPhaseIdentifier, Collider* collider, const AABB& aabb, bool forceInsert) {
  assert(broadPhaseIdentifier >= 0);
  /* Update the dynamic tree */
  bool reinsert = mDynamicTree.update(broadPhaseIdentifier, aabb, forceInsert);

  /* Shape has moved out of the bound of its fat AABB which means it has been reinserted into the tree */
  if(reinsert) {
    /* Mark the shape as having moved in the previous frame */
    addColliderForTest(broadPhaseIdentifier, collider);
  }
}

/* Update broad phase state of select collider components */
void BroadPhase::updateColliderComponents(uint32 start, uint32 numComponents) {
  assert(numComponents > 0);
  assert(start < mColliderComponents.getNumComponents() && start + numComponents <= mColliderComponents.getNumComponents());
  /* Leave disabled components alone */
  start = std::min(start, mColliderComponents.getNumEnabledComponents());
  uint32 end = std::min(start + numComponents, mColliderComponents.getNumEnabledComponents());
  numComponents = end - start;

  for(uint32 i = start; i < start + numComponents; i++) {
    const int32 broadPhaseIdentifier = mColliderComponents.mBroadPhaseIdentifiers[i];

    if(broadPhaseIdentifier != -1) {
      const Entity& bodyEntity = mColliderComponents.mBodyEntities[i];
      const Transform& bodyTransform = mTransformComponents.getTransform(bodyEntity);
      /* Recompute the world space AABB */
      AABB aabb;
      mColliderComponents.mShapes[i]->computeAABB(aabb, bodyTransform * mColliderComponents.mTransformsLocalBody[i]);
      /* If the geometry of the collider shape has been changed, we need to reset the AABB in broad phase accordingly */
      const bool forceInsert = mColliderComponents.mHasSizeChanged[i];
      /* Notify broad phase about this change */
      notifyColliderUpdate(broadPhaseIdentifier, mColliderComponents.mColliders[i], aabb, forceInsert);
      mColliderComponents.mHasSizeChanged[i] = false;
    }
  }
}

/* Add collider */
void BroadPhase::addCollider(Collider* collider, const AABB& aabb) {
  assert(collider->getBroadPhaseIdentifier() == -1);
  /* Insert the collider into the dynamic tree and get the broad phase identifier */
  int32 nodeIdentifier = mDynamicTree.add(aabb, collider);
  /* Assign the broad phase identifier */
  mColliderComponents.setBroadPhaseIdentifier(collider->getEntity(), nodeIdentifier);
  /* Mark the shape as having moved in the previous frame */
  addColliderForTest(collider->getBroadPhaseIdentifier(), collider);
}

/* Remove collider */
void BroadPhase::removeCollider(Collider* collider) {
  assert(collider->getBroadPhaseIdentifier() != -1);
  int32 broadPhaseIdentifier = collider->getBroadPhaseIdentifier();
  mColliderComponents.setBroadPhaseIdentifier(collider->getEntity(), -1);
  /* Remove the collider from the dynamic tree */
  mDynamicTree.remove(broadPhaseIdentifier);
  /* Unmark the shape as having moved in the previous frame */
  removeColliderForTest(broadPhaseIdentifier);
}

/* Update collider */
void BroadPhase::updateCollider(Entity entity) {
  assert(mColliderComponents.mEntityComponentMap.contains(entity));
  /* Index of the collider component in the collider components array */
  uint32 colliderIndex = mColliderComponents.mEntityComponentMap[entity];
  updateColliderComponents(colliderIndex, 1);
}

/* Update all enabled colliders */
void BroadPhase::updateColliders() {
  /* Only update enabled collider components */
  if(mColliderComponents.getNumEnabledComponents()) {
    updateColliderComponents(0, mColliderComponents.getNumEnabledComponents());
  }
}

/* Add collider to be testerd for overlap */
void BroadPhase::addColliderForTest(int32 broadPhaseIdentifier, Collider* collider) {
  assert(broadPhaseIdentifier != -1);
  /* Add the broad phase identifier to the array of existing identifiers to be tested */
  mShapesToTest.insert(broadPhaseIdentifier);
  /* Overlap pairs which contain the current shape need to be re-tested */
  mCollisionDetection.notifyOverlapPairsToTest(collider);
}

/* Remove collider to be tested for overlap */
void BroadPhase::removeColliderForTest(int32 broadPhaseIdentifier) {
  /* Remove the broad phase identifier from the array of existing identifiers to be tested */
  mShapesToTest.remove(broadPhaseIdentifier);
}

/* Get the collider associated with the provided broad phase identifier */
Collider* BroadPhase::getCollider(int32 broadPhaseIdentifier) const {
  return static_cast<Collider*>(mDynamicTree.getNodeData(broadPhaseIdentifier));
}

/* Compute overlap pairs */
void BroadPhase::computeOverlapPairs(MemoryStrategy& memoryStrategy, DynamicArray<Pair<int32, int32>>& overlapNodes) {
  /* All colliders that have been marked as having moved in the previous frame */
  DynamicArray<int32> shapesToTest = mShapesToTest.toArray(memoryStrategy.getFreeListMemoryHandler());
  /* Use the dynamic structure to determine all shapes which overlap with the shapes of those colliders that have moved in the previous frame */
  mDynamicTree.getShapeShapeOverlaps(shapesToTest, 0, static_cast<uint32>(shapesToTest.size()), overlapNodes);
  mShapesToTest.clear();
}

/* Query whether two shapes are overlapping */
bool BroadPhase::testShapesOverlap(int32 firstBroadPhaseIdentifier, int32 secondBroadPhaseIdentifier) const {
  assert(firstBroadPhaseIdentifier != -1);
  assert(secondBroadPhaseIdentifier != -1);
  /* Need to obtain the AABBs to test possible overlap in broad phase */
  const AABB& firstAABB = mDynamicTree.getFatAABB(firstBroadPhaseIdentifier);
  const AABB& secondAABB = mDynamicTree.getFatAABB(secondBroadPhaseIdentifier);
  /* Use overlap test API of the AABB */
  return firstAABB.isOverlapping(secondAABB);
}

/* Get fat AABB of the shape associated with the provided broad phase identifier */
const AABB& BroadPhase::getFatAABB(int32 broadPhaseIdentifier) {
  return mDynamicTree.getFatAABB(broadPhaseIdentifier);
}