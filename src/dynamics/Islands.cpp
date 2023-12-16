#include <physics/dynamics/Islands.h>

using namespace physics;

/* Constructor */
Islands::Islands(MemoryHandler& memoryHandler) :
                 mNumIslandsLastFrame(16),
                 mNumBodiesLastFrame(32),
                 mMaxNumBodiesLastFrame(0),
                 mMaxNumBodiesCurrentFrame(0),
                 mManifoldIslandMap(memoryHandler),
                 manifoldIndices(memoryHandler),
                 numManifolds(memoryHandler),
                 bodies(memoryHandler),
                 bodyIndices(memoryHandler),
                 numBodies(memoryHandler),
                 solved(memoryHandler) {}

/* Get the number of islands */
uint32 Islands::getNumIslands() const {
  return static_cast<uint32>(manifoldIndices.size());
}

/* Get the island index from the given manifold start index */
uint32 Islands::getIslandIndex(uint32 manifoldStartIndex) const {
  assert(mManifoldIslandMap.contains(manifoldStartIndex));
  return mManifoldIslandMap[manifoldStartIndex];
}

/* Add an island */
uint32 Islands::addIsland(uint32 manifoldStartIndex) {
  const uint32 islandIndex = static_cast<uint32>(manifoldIndices.size());
  mManifoldIslandMap.insert(Pair<uint32, uint32>(manifoldStartIndex, islandIndex));
  manifoldIndices.add(manifoldStartIndex);
  numManifolds.add(0);
  bodyIndices.add(static_cast<uint32>(bodies.size()));
  numBodies.add(0);
  solved.add(true);

  if(islandIndex > 0 && numBodies[islandIndex - 1] > mMaxNumBodiesCurrentFrame) {
    mMaxNumBodiesCurrentFrame = numBodies[islandIndex - 1];
  }

  return islandIndex;
}

/* Add a body to an island */
void Islands::addBody(Entity bodyEntity) {
  const uint32 islandIndex = static_cast<uint32>(manifoldIndices.size());
  assert(islandIndex > 0);
  bodies.add(bodyEntity);
  numBodies[islandIndex - 1]++;
}

/* Reserve memory for current frame  */
void Islands::reserve() {
  mManifoldIslandMap.reserve(mNumIslandsLastFrame);
  manifoldIndices.reserve(mNumIslandsLastFrame);
  numManifolds.reserve(mNumIslandsLastFrame);
  bodyIndices.reserve(mNumIslandsLastFrame);
  numBodies.reserve(mNumIslandsLastFrame);
  bodies.reserve(mNumBodiesLastFrame);
  solved.reserve(mNumIslandsLastFrame);
}

/* Clear all islands */
void Islands::clear() {
  const uint32 numIslands = static_cast<uint32>(numManifolds.size());

  if(numIslands > 0 && numBodies[numIslands - 1] > mMaxNumBodiesCurrentFrame) {
    mMaxNumBodiesCurrentFrame = numBodies[numIslands - 1];
  }

  mMaxNumBodiesLastFrame = mMaxNumBodiesCurrentFrame;
  mNumIslandsLastFrame = numIslands;
  mMaxNumBodiesCurrentFrame = 0;
  mNumBodiesLastFrame = static_cast<uint32>(bodies.size());
  mManifoldIslandMap.clear(true);
  manifoldIndices.clear(true);
  numManifolds.clear(true);
  bodies.clear(true);
  bodyIndices.clear(true);
  numBodies.clear(true);
  solved.clear(true);
}