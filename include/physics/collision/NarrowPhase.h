#ifndef PHYSICS_NARROW_PHASE_H
#define PHYSICS_NARROW_PHASE_H

#include <physics/Configuration.h>
#include <physics/collision/OverlapPairs.h>
#include <physics/collision/Contact.h>

namespace physics {

/* Forward declarations */
class CollisionAlgorithm;

class NarrowPhase {

  private:
    /* -- Attributes -- */

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* Broad phase overlap pairs */
    OverlapPairs& mOverlapPairs;

    /* Cached capacity */
    uint32 mCachedCapacity = 0;

  public:
    /* -- Nested classes -- */

    /* Input information to the narrow phase module */
    struct NarrowPhaseInfo {
      
      public:
        /* -- Attributes -- */

        /* Overlap pairs identifiers from broadphase */
        uint64 overlapPairIdentifier;

        /* Entity of the first collider */
        Entity firstColliderEntity;

        /* Entity of the second collider */
        Entity secondColliderEntity;

        /* First collision shape */
        Shape* firstShape;

        /* Second collision shape */
        Shape* secondShape;

        /* Local to world transform of the first shape */
        Transform firstShapeTransform;

        /* Local to world transform of the second shape */
        Transform secondShapeTransform;

        /* Collision algorithm based on shape type */
        CollisionAlgorithm* algorithm;

        /* Result of the collision detection test in narrow phase */
        bool isColliding;

        /* -- Methods -- */

        /* Constructor */
        NarrowPhaseInfo(uint64 overlapPairIdentifier,
                        Entity firstColliderEntity,
                        Entity secondColliderEntity,
                        Shape* firstShape,
                        Shape* secondShape,
                        Transform firstShapeTransform,
                        Transform secondShapeTransform,
                        CollisionAlgorithm* algorithm) :
                        overlapPairIdentifier(overlapPairIdentifier),
                        firstColliderEntity(firstColliderEntity),
                        secondColliderEntity(secondColliderEntity),
                        firstShape(firstShape),
                        secondShape(secondShape),
                        firstShapeTransform(firstShapeTransform),
                        secondShapeTransform(secondShapeTransform),
                        algorithm(algorithm),
                        isColliding(false) {}
    };

  /* -- Attributes -- */

  /* Narrow phase entries */
  DynamicArray<NarrowPhaseInfo> entries;

  /* -- Methods -- */
  
  /* Constructor */
  NarrowPhase(OverlapPairs& overlapPairs, MemoryHandler& memoryHandler);

  /* Destructor */
  ~NarrowPhase();

  /* Add narrow phase entry */
  void addEntry(uint64 overlapPairIdentifier,
                Entity firstColliderEntity,
                Entity secondColliderEntity,
                Shape* firstShape,
                Shape* secondShape,
                const Transform& firstShapeTransform,
                const Transform& secondShapeTransform,
                CollisionAlgorithm* algorithm);

  /* Initialize using cached capacity */
  void reserve();

  /* Clear all entries */
  void clear();
};

}

#endif