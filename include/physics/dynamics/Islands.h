#ifndef PHYSICS_ISLANDS_H
#define PHYSICS_ISLANDS_H

#include <physics/Configuration.h>
#include <physics/collections/DynamicArray.h>
#include <physics/collections/Map.h>
#include <physics/common/Entity.h>

namespace physics {

/* In this context, an island is a group of bodies that are connected to each other via contacts */
class Islands {

  private:
    /* -- Attributes -- */
    
    /* Number of islands in the last frame */
    uint32 mNumIslandsLastFrame;

    /* Number of bodies in the last frame */
    uint32 mNumBodiesLastFrame;

    /* Maximum number of bodies in a particular island in the last frame */
    uint32 mMaxNumBodiesLastFrame;

    /* Maximum number of bodies in a particular island in the current frame */
    uint32 mMaxNumBodiesCurrentFrame;

    /* Map manifold index to island index */
    Map<uint32, uint32> mManifoldIslandMap;

  public:
    /* -- Attributes -- */

    /* Array of manifold start indices for each island */
    DynamicArray<uint32> manifoldIndices;

    /* Number of manifolds for each island */
    DynamicArray<uint32> numManifolds;

    /* Array of all bodies */
    DynamicArray<Entity> bodies;

    /* Array of body start indices for each island */
    DynamicArray<uint32> bodyIndices;

    /* Number of bodies for each island */
    DynamicArray<uint32> numBodies;

    /* Solver results for each island */
    DynamicArray<bool> solved;

    /* -- Methods -- */

    /* Constructor */
    Islands(MemoryHandler& memoryHandler);

    /* Destructor */
    ~Islands() = default;

    /* Copy constructor */
    Islands(const Islands& islands) = default;

    /* Deleted assignment operator */
    Islands& operator=(const Islands& islands) = delete;

    /* Get the number of islands */
    uint32 getNumIslands() const;

    /* Get the island index from the given manifold start index */
    uint32 getIslandIndex(uint32 manifoldStartIndex) const;

    /* Add an island */
    uint32 addIsland(uint32 manifoldStartIndex);
    
    /* Add a body to an island */
    void addBody(Entity bodyEntity);

    /* Reserve memory for current frame */
    void reserve();

    /* Clear all islands */
    void clear();
};

}

#endif