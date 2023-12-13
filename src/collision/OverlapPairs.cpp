#include <physics/collision/OverlapPairs.h>
#include <physics/collision/algorithms/AlgorithmDispatch.h>

using namespace physics;

/* Constructor */
OverlapPairs::OverlapPairs(MemoryStrategy& memoryStrategy,
                           BodyComponents& bodyComponents,
                           ColliderComponents& colliderComponents,
                           Set<Pair<Entity, Entity>>& incompatibleCollisionPairs,
                           AlgorithmDispatch& algorithmDispatch) :
                           mPoolHandler(memoryStrategy.getObjectPoolMemoryHandler()),
                           mFreeListHandler(memoryStrategy.getFreeListMemoryHandler()),
                           mPairs(memoryStrategy.getFreeListMemoryHandler()),
                           mPairIdentifierArrayIndexMap(memoryStrategy.getFreeListMemoryHandler()),
                           mBodyComponents(bodyComponents),
                           mColliderComponents(colliderComponents),
                           mIncompatibleCollisionPairs(incompatibleCollisionPairs),
                           mAlgorithmDispatch(algorithmDispatch) {}

/* Destructor */
OverlapPairs::~OverlapPairs() {
  /* Remove all overlap pairs from broadphase and release their memory */
  while(mPairs.size()) {
    removeOverlapPair(mPairs.size() - 1);
  }
}

/* Add overlap pair */
uint64 OverlapPairs::addOverlapPair(uint32 firstColliderIndex, uint32 secondColliderIndex) {
  assert(mColliderComponents.mBroadPhaseIdentifiers[firstColliderIndex] >= 0);
  assert(mColliderComponents.mBroadPhaseIdentifiers[secondColliderIndex] >= 0);
  const Shape* firstShape = mColliderComponents.mShapes[firstColliderIndex];
  const Shape* secondShape = mColliderComponents.mShapes[secondColliderIndex];
  const Entity firstColliderEntity = mColliderComponents.mColliderEntities[firstColliderIndex];
  const Entity secondColliderEntity = mColliderComponents.mColliderEntities[secondColliderIndex];
  const uint32 firstBroadPhaseIdentifier = static_cast<uint32>(mColliderComponents.mBroadPhaseIdentifiers[firstColliderIndex]);
  const uint32 secondBroadPhaseIdentifier = static_cast<uint32>(mColliderComponents.mBroadPhaseIdentifiers[secondColliderIndex]);
  /* Create a unique identifier for the overlap pair */
  const uint64 pairIdentifier = getElegantPair(firstBroadPhaseIdentifier, secondBroadPhaseIdentifier);
  assert(!mPairIdentifierArrayIndexMap.contains(pairIdentifier));

  /* Decide the collision algorithm based on the two collision shape types */
  CollisionAlgorithmType algorithmType = mAlgorithmDispatch.getCollisionAlgorithmType(firstShape->getType(), secondShape->getType());
  /* Map identifier to index in the array of overlap pairs  */
  mPairIdentifierArrayIndexMap.insert(Pair<uint64, uint64>(pairIdentifier, mPairs.size()));
  /* Create new pair and append it to the array */
  mPairs.emplace(pairIdentifier, firstBroadPhaseIdentifier, secondBroadPhaseIdentifier, firstColliderEntity, secondColliderEntity, algorithmType);
  assert(mColliderComponents.mOverlapPairs[firstColliderIndex].find(pairIdentifier) == mColliderComponents.mOverlapPairs[firstColliderIndex].end());
  assert(mColliderComponents.mOverlapPairs[secondColliderIndex].find(pairIdentifier) == mColliderComponents.mOverlapPairs[secondColliderIndex].end());
  /* Associate overlap pairs with the colliders of each body */
  mColliderComponents.mOverlapPairs[firstColliderIndex].add(pairIdentifier);
  mColliderComponents.mOverlapPairs[secondColliderIndex].add(pairIdentifier);
  return pairIdentifier;
}

/* Erase overlap pair */
void OverlapPairs::eraseOverlapPair(uint64 pairIdentifier) {
  assert(mPairIdentifierArrayIndexMap.contains(pairIdentifier));
  /* Confirm that the overlap pair exists before we attempt to remove it */
  auto iter = mPairIdentifierArrayIndexMap.find(pairIdentifier);

  if(iter != mPairIdentifierArrayIndexMap.end()) {
    removeOverlapPair(iter->second);
  }
}

/* Remove overlap pair */
void OverlapPairs::removeOverlapPair(uint64 pairIndex) {
  const uint64 numPairs = mPairs.size();
  assert(pairIndex < numPairs);
  assert(mColliderComponents.getOverlapPairs(mPairs[pairIndex].firstColliderEntity).find(mPairs[pairIndex].pairIdentifier) != mColliderComponents.getOverlapPairs(mPairs[pairIndex].firstColliderEntity).end());
  assert(mColliderComponents.getOverlapPairs(mPairs[pairIndex].secondColliderEntity).find(mPairs[pairIndex].pairIdentifier) != mColliderComponents.getOverlapPairs(mPairs[pairIndex].secondColliderEntity).end());
  assert(mPairIdentifierArrayIndexMap[mPairs[pairIndex].pairIdentifier] == pairIndex);

  /* Remove index from overlap pairs arrays and map */
  mColliderComponents.getOverlapPairs(mPairs[pairIndex].firstColliderEntity).remove(mPairs[pairIndex].pairIdentifier);
  mColliderComponents.getOverlapPairs(mPairs[pairIndex].secondColliderEntity).remove(mPairs[pairIndex].pairIdentifier);
  mPairIdentifierArrayIndexMap.remove(mPairs[pairIndex].pairIdentifier);

  /* Replace the removed pair with the last one in the array */
  if(mPairs.size() > 1 && pairIndex < (numPairs - 1)) {
    mPairIdentifierArrayIndexMap[mPairs[numPairs - 1].pairIdentifier] = pairIndex;
  }

  mPairs[pairIndex] = mPairs[numPairs - 1];
  mPairs.erase(numPairs - 1);
}

/* Get body index pair */
typename Pair<Entity, Entity> OverlapPairs::getBodyIndexPair(Entity firstBodyEntity, Entity secondBodyEntity) {
  /* Construct pair */
  Pair<Entity, Entity> indexPair = firstBodyEntity.identifier < secondBodyEntity.identifier ? Pair<Entity, Entity>(firstBodyEntity, secondBodyEntity) : Pair<Entity, Entity>(secondBodyEntity, firstBodyEntity);
  assert(indexPair.first != indexPair.second);
  return indexPair;
}

/* Set test for overlap */
void OverlapPairs::setTestOverlap(uint64 pairIdentifier, bool testOverlap) {
  assert(mPairIdentifierArrayIndexMap.contains(pairIdentifier));
  /* Confirm that the overlap pair exists before attempting to test for overlap */
  auto iter = mPairIdentifierArrayIndexMap.find(pairIdentifier);

  if(iter != mPairIdentifierArrayIndexMap.end()) {
    mPairs[static_cast<uint32>(iter->second)].testOverlap = testOverlap;
  }
}

/* Get pointer to overlap pair */
typename OverlapPairs::OverlapPair* OverlapPairs::getOverlapPair(uint64 pairIdentifier) {
  /* Confirm that the overlap pair exists before attempting to return its pointer */
  auto iter = mPairIdentifierArrayIndexMap.find(pairIdentifier);

  if(iter != mPairIdentifierArrayIndexMap.end()) {
    return &(mPairs[static_cast<uint32>(iter->second)]);
  }

  return nullptr;
}