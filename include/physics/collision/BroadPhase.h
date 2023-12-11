#ifndef PHYSICS_BROAD_PHASE_H
#define PHYSICS_BROAD_PHASE_H

#include <physics/collision/DynamicTree.h>
#include <physics/collections/List.h>
#include <physics/collections/set.h>
#include <physics/common/TransformComponents.h>
#include <physics/common/ColliderComponents.h>
#include <physics/common/BodyComponents.h>

namespace physics {

/* Forward declarations */
class MemoryStrategy;
class BroadPhase;
class CollisionDetection;
class Body;
class Collider;

class BroadPhase {

  private:
    /* -- Attributes -- */

    /* Dynamic Tree */
    DynamicTree mDynamicTree;

    /* Body components */
    BodyComponents& mBodyComponents;

    /* Collider components */
    ColliderComponents& mColliderComponents;

    /* Transform components */
    TransformComponents& mTransformComponents;

    /* Shapes to test */
    Set<int32> mShapesToTest;

    /* Collision Detection */
    CollisionDetection& mCollisionDetection;

    /* -- Methods -- */

    /* Notify tree about collider update */
    void notifyColliderUpdate(int32 broadPhaseIdentifier, Collider* collider, const AABB& aabb, bool forceInsert);

    /* Update broad phase state of select collider components */
    void updateColliderComponents(uint32 start, uint32 numComponents);
  
  public:
    /* -- Methods -- */

    /* Constructor */
    BroadPhase(CollisionDetection& collisionDetection, BodyComponents& bodyComponents, ColliderComponents& colliderComponents, TransformComponents& transformComponents);

    /* Destructor */
    ~BroadPhase() = default;

    /* Deleted copy constructor */
    BroadPhase(const BroadPhase& broadPhase) = delete;

    /* Deleted assignment operator */
    BroadPhase& operator=(const BroadPhase& broadPhase) = delete;

    /* Add collider */
    void addCollider(Collider* collider, const AABB& aabb);

    /* Remove collider */
    void removeCollider(Collider* collider);

    /* Update collider */
    void updateCollider(Entity entity);

    /* Update all enabled colliders */
    void updateColliders();

    /* Add collider to be tested for overlap */
    void addColliderForTest(int32 broadPhdaseIdentifier, Collider* collider);

    /* Remove collider to be tested for overlap */
    void removeColliderForTest(int32 broadPhaseIdentifier);

    /* Get the collider associated with the provided broad phase identifier */
    Collider* getCollider(int32 broadPhaseIdentifier) const;

    /* Compute overlap pairs */
    void computeOverlapPairs(MemoryStrategy& memoryStrategy, DynamicArray<Pair<int32, int32>>& overlapNodes);

    /* Query whether two shapes are overlapping */
    bool testShapesOverlap(int32 firstBroadPhaseIdentifier, int32 secondBroadPhaseIdentifier) const;

    /* Get fat AABB of the shape associated with the provided broad phase identifier */
    const AABB& getFatAABB(int32 broadPhaseIdentifier);
};

}

#endif