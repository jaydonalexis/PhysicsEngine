#ifndef PHYSICS_COLLISION_DETECTION_H
#define PHYSICS_COLLISION_DETECTION_H

#include <physics/collision/Contact.h>
#include <physics/common/BodyComponents.h>
#include <physics/common/ColliderComponents.h>
#include <physics/common/TransformComponents.h>
#include <physics/collision/BroadPhase.h>
#include <physics/collision/OverlapPairs.h>
#include <physics/collision/NarrowPhase.h>
#include <physics/collision/algorithms/AlgorithmDispatch.h>

namespace physics {

/* Forward declarations */
class World;
class MemoryStrategy;
class AlgorithmDispatch;

class CollisionDetection {

  private:
    /* -- Attributes -- */

    /* Pointer to the world */
    World* mWorld;

    /* Memory handler */
    MemoryStrategy& mMemoryStrategy;

    /* Body components */
    BodyComponents& mBodyComponents;

    /* Collider components */
    ColliderComponents& mColliderComponents;

    /* Transform components */
    TransformComponents& mTransformComponents;

    /* Overlapping nodes in broad phase */
    DynamicArray<Pair<int32, int32>> mBroadPhaseOverlapNodes;

    /* Incompatible collision pairs */
    Set<Pair<Entity, Entity>> mIncompatibleCollisionPairs;

    /* Identifier-Entity map */
    Map<int32, Entity> mIdentifierEntityMap;

    /* Collision algorithm dispatch */
    AlgorithmDispatch mAlgorithmDispatch;

    /* Map overlap pair identifier to contact pair index in the last frame */
    Map<uint64, uint32> mOverlapPairLastContactPairMap;

    /* First contact pair set */
    DynamicArray<ContactPair> mContactPairsA;

    /* Second contact pair set */
    DynamicArray<ContactPair> mContactPairsB;

    /* Current contact pairs */
    DynamicArray<ContactPair>* mCurrentContactPairs;

    /* Previous contact pairs */
    DynamicArray<ContactPair>* mLastContactPairs;

    /* First contact manifold set */
    DynamicArray<LocalManifold> mManifoldsA;

    /* Second contact manifold set */
    DynamicArray<LocalManifold> mManifoldsB;

    /* Raw contact manifolds */
    DynamicArray<LocalManifold> mRawManifolds;
    
    /* Current contact manifolds */
    DynamicArray<LocalManifold>* mCurrentManifolds;

    /* Previous contact manifolds */
    DynamicArray<LocalManifold>* mLastManifolds;

    /* Broad phase */
    BroadPhase mBroadPhase;

    /* Overlap pairs in broad phase */
    OverlapPairs mOverlapPairs;

    /* Narrow phase */
    NarrowPhase mNarrowPhase;

    /* -- Methods -- */

    /* Compute broad phase collision detection */
    void runBroadPhase();

    /* Prepare for narrow phase collision detection*/
    void prepareNarrowPhase(NarrowPhase& narrowPhase);

    /* Compute narrow phase collision detection */
    void runNarrowPhase();

    /* Exchange frame info */
    void exchangeFrameInfo();

    /* Update overlap pairs with overlapping nodes from broad phase */
    void updateOverlapPairs(const DynamicArray<Pair<int32, int32>>& overlapNodes);

    /* Remove pairs that are not overlapping anymore */
    void removeOverlapPairs();

    /* Filter overlap pairs where only a given body is involved */
    void filterOverlapPairs(Entity bodyEntity, DynamicArray<uint64>& pairs);

    /* Filter overlap pairs where two given bodies are involved */
    void filterOverlapPairs(Entity firstBodyEntity, Entity secondBodyEntity, DynamicArray<uint64>& pairs);

    /* Process narrow phase input */
    void processNarrowPhase(NarrowPhase& narrowPhase, DynamicArray<ContactPair>* contactPairs, DynamicArray<LocalManifold>& manifolds);

    /* Add the contact pairs to the appropriate bodies */
    void associateContactPairs();

    /* Prepare collision detection results for the contact solver */
    void prepareForContactSolver();

    /* Update the current contact information with those of the last frame for warm starting */
    void prepareForWarmStart();

    /* Populate the overlap pair identifier to last frame contact pair index map */
    void populateLastContactPairMap();

  public:
    /* -- Methods -- */

    /* Constructor */
    CollisionDetection(World* world, MemoryStrategy& memoryStrategy, BodyComponents& bodyComponents, ColliderComponents& colliderComponents, TransformComponents& transformComponents);

    /* Destructor */
    ~CollisionDetection() = default;

    /* Deleted copy constructor */
    CollisionDetection(const CollisionDetection& collisionDetection) = delete;

    /* Deleted assignment operator */
    CollisionDetection& operator=(const CollisionDetection& collisionDetection) = delete;

    /* Execute collision detection */
    void execute();

    /* Add collider to the collision detection system */
    void addCollider(Collider* collider, const AABB& aabb);

    /* Remove collider from the collision detection system */
    void removeCollider(Collider* collider);

    /* Update a collider in the collision detection system */
    void updateCollider(Entity entity);

    /* Update all colliders in the collision detection system */
    void updateColliders();

    /* Add body pair that are incompatible for collision */
    void addIncompatibleCollisionPair(Entity firstBodyEntity, Entity secondBodyEntity);

    /* Remove body pair that are incompatible for collision */
    void removeIncompatibleCollisionPair(Entity firstBodyEntity, Entity secondBodyEntity);

    /* Forcibly test shape for broad phase collision */
    void checkBroadPhaseCollision(Collider* collider);
    
    /* Notify that overlap pairs where the given collider is involved need to be tested for overlap */
    void notifyOverlapPairsToTest(Collider* collider);

    /* Get collision algorithm dispatch */
    AlgorithmDispatch& getAlgorithmDispatch();

    /* Return a reference to memory strategy */
    MemoryStrategy& getMemoryStrategy() const;

    /* -- Friends -- */

    friend class World;
    friend class Body;
};

}

#endif