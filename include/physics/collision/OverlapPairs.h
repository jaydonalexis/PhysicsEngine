#ifndef PHYSICS_OVERLAP_PAIRS_H
#define PHYSICS_OVERLAP_PAIRS_H

#include <physics/collision/Collider.h>
#include <physics/common/BodyComponents.h>
#include <physics/common/ColliderComponents.h>
#include <physics/common/TransformComponents.h>
#include <physics/collections/Map.h>
#include <physics/collections/Set.h>
#include <physics/collections/Pair.h>
#include <physics/memory/MemoryStrategy.h>

namespace physics {

/* Forward declarations */
enum class CollisionAlgorithmType;
class NarrowPhase;
class AlgorithmDispatch;
class Shape;

class OverlapPairs {

  public:
    /* -- Nested Classes -- */
    
    /* Single overlap pair */
    struct OverlapPair {
      
      public:
        /* -- Attributes -- */

        /* Overlap pair identifier */
        uint64 pairIdentifier;

        /* Broad phase identifier of first shape */
        int32 firstBroadPhaseIdentifier;

        /* Broad phase identifier of the second shape */
        int32 secondBroadPhaseIdentifier;

        /* Entity of the collider of the first shape */
        Entity firstColliderEntity;

        /* Entity of the collider of the second shape */
        Entity secondColliderEntity;

        /* Test if overlap pairs of shapes from the previous frame still overlap */
        bool testOverlap;

        CollisionAlgorithmType collisionAlgorithmType;

        /* -- Methods -- */

        /* Constructor */
        OverlapPair(uint64 pairIdentifier,
                    int32 firstBroadPhaseIdentifier,
                    int32 secondBroadPhaseIdentifier,
                    Entity firstColliderEntity,
                    Entity secondColliderEntity,
                    CollisionAlgorithmType collisionAlgorithmType) :
                    pairIdentifier(pairIdentifier),
                    firstBroadPhaseIdentifier(firstBroadPhaseIdentifier),
                    secondBroadPhaseIdentifier(secondBroadPhaseIdentifier),
                    firstColliderEntity(firstColliderEntity),
                    secondColliderEntity(secondColliderEntity),
                    testOverlap(false),
                    collisionAlgorithmType(collisionAlgorithmType) {}
    };

  private:
    /* -- Attributes -- */
    
    /* Object pool memory handler */
    MemoryHandler& mPoolHandler;

    /* Free list memory handler */
    MemoryHandler& mFreeListHandler;

    /* Overlap pairs */
    DynamicArray<OverlapPair> mPairs;

    /* Pair identifier to array index map */
    Map<uint64, uint64> mPairIdentifierArrayIndexMap;

    /* Body components */
    BodyComponents& mBodyComponents;

    /* Collider components */
    ColliderComponents& mColliderComponents;

    /* Bodies that are now allowed to collide */
    Set<Pair<Entity, Entity>>& mIncompatibleCollisionPairs;

    /* Collision algorithm dispatch */
    AlgorithmDispatch& mAlgorithmDispatch;

  public:
    /* -- Methods -- */

    /* Constructor */
    OverlapPairs(MemoryStrategy& memoryStrategy,
                 BodyComponents& bodyComponents,
                 ColliderComponents& colliderComponents,
                 Set<Pair<Entity, Entity>>& incompatibleCollisionPairs,
                 AlgorithmDispatch& algorithmDispatch);

    /* Destructor */
    ~OverlapPairs();

    /* Deleted copy constructor */
    OverlapPairs(const OverlapPairs& overlapPairs) = delete;

    /* Deleted assignment operator */
    OverlapPairs& operator=(const OverlapPairs& overlapPairs) = delete;

    /* Add overlap pair */
    uint64 addOverlapPair(uint32 firstColliderIndex, uint32 secondColliderIndex);

    /* Erase overlap pair */
    void eraseOverlapPair(uint64 pairIdentifier);

    /* Remove overlap pair */
    void removeOverlapPair(uint64 pairIndex);

    /* Get body pair index */
    static Pair<Entity, Entity> getBodyIndexPair(Entity firstBodyEntity, Entity secondBodyEntity);

    /* Set test for overlap */
    void setTestOverlap(uint64 pairIdentifier, bool testOverlap);

    /* Get pointer to overlap pair */
    OverlapPair* getOverlapPair(uint64 pairIdentifier);

    /* -- Friends -- */
    friend class CollisionDetection;
};

}

#endif