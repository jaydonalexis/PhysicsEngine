#include <physics/collision/CollisionDetection.h>
#include <physics/common/World.h>
#include <physics/memory/MemoryStrategy.h>
#include <physics/common/Factory.h>

using namespace physics;

/* Constructor */
CollisionDetection::CollisionDetection(World* world,
                                       MemoryStrategy& memoryStrategy,
                                       BodyComponents& bodyComponents,
                                       ColliderComponents& colliderComponents,
                                       TransformComponents& transformComponents) :
                                       mWorld(world),
                                       mMemoryStrategy(memoryStrategy),
                                       mBodyComponents(bodyComponents),
                                       mColliderComponents(colliderComponents),
                                       mTransformComponents(transformComponents),
                                       mBroadPhaseOverlapNodes(mMemoryStrategy.getFreeListMemoryHandler(), 32),
                                       mIncompatibleCollisionPairs(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mIdentifierEntityMap(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mAlgorithmDispatch(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mOverlapPairLastContactPairMap(mMemoryStrategy.getFreeListMemoryHandler()),
                                       mContactPairsA(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mContactPairsB(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mLastContactPairs(&mContactPairsA),
                                       mCurrentContactPairs(&mContactPairsB),
                                       mManifoldsA(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mManifoldsB(mMemoryStrategy.getObjectPoolMemoryHandler()),
                                       mLastManifolds(&mManifoldsA),
                                       mCurrentManifolds(&mManifoldsB),
                                       mRawManifolds(mMemoryStrategy.getLinearMemoryHandler()),
                                       mBroadPhase(*this,
                                                    mBodyComponents, 
                                                    mColliderComponents, 
                                                    mTransformComponents),
                                       mOverlapPairs(mMemoryStrategy,
                                                     mBodyComponents,
                                                     mColliderComponents,
                                                     mIncompatibleCollisionPairs,
                                                     mAlgorithmDispatch),
                                       mNarrowPhase(mOverlapPairs,
                                                    mMemoryStrategy.getLinearMemoryHandler()) {}

/* Compute broad phase collision detection */
void CollisionDetection::runBroadPhase() {
  assert(!mBroadPhaseOverlapNodes.size());
  /* Use dynamic tree to find all shapes overlapping with those that have moved in the previous frame */
  mBroadPhase.computeOverlapPairs(mMemoryStrategy, mBroadPhaseOverlapNodes);
  LOG(std::to_string(mBroadPhaseOverlapNodes.size()) + " overlapping node(s) found");
  /* Create new overlap pairs */
  updateOverlapPairs(mBroadPhaseOverlapNodes);
  /* Remove overlap pairs which are not overlapping anymore */
  removeOverlapPairs();
  mBroadPhaseOverlapNodes.clear();
}

void CollisionDetection::prepareNarrowPhase(NarrowPhase& narrowPhase) {
  /* Use cache to reserve sufficient memory for the narrow phase entries */
  narrowPhase.reserve();
  const uint32 numPairs = static_cast<uint32>(mOverlapPairs.mPairs.size());
  LOG("Retrieved " + std::to_string(numPairs) + " overlap pair(s) from broad phase");

  for(uint32 i = 0; i < numPairs; i++) {
    OverlapPairs::OverlapPair& overlapPair = mOverlapPairs.mPairs[i];
    assert(mColliderComponents.getBroadPhaseIdentifier(overlapPair.firstColliderEntity) != -1);
    assert(mColliderComponents.getBroadPhaseIdentifier(overlapPair.secondColliderEntity) != -1);
    assert(mColliderComponents.getBroadPhaseIdentifier(overlapPair.firstColliderEntity) != mColliderComponents.getBroadPhaseIdentifier(overlapPair.secondColliderEntity));
    const Entity firstColliderEntity = overlapPair.firstColliderEntity;
    const Entity secondColliderEntity = overlapPair.secondColliderEntity;
    const uint32 firstColliderIndex = mColliderComponents.getComponentEntityIndex(firstColliderEntity);
    const uint32 secondColliderIndex = mColliderComponents.getComponentEntityIndex(secondColliderEntity);
    Shape* firstShape = mColliderComponents.mShapes[firstColliderIndex];
    Shape* secondShape = mColliderComponents.mShapes[secondColliderIndex];
    CollisionAlgorithmType algorithmType = overlapPair.collisionAlgorithmType;

    std::stringstream ss;
    ss << " First Index: " << firstColliderEntity.getIndex() << ", Second Index: " << secondColliderEntity.getIndex() << ", Algorithm Type: " << (int)algorithmType;
    LOG("Overlap pair " + std::to_string(i) + ss.str());

    /* Add an entry for the current broadphase overlap pair into the narrow phase */
    narrowPhase.addEntry(overlapPair.pairIdentifier,
                         firstColliderEntity,
                         secondColliderEntity,
                         firstShape,
                         secondShape,
                         mColliderComponents.mTransformsLocalWorld[firstColliderIndex],
                         mColliderComponents.mTransformsLocalWorld[secondColliderIndex],
                         mAlgorithmDispatch.getCollisionAlgorithm(algorithmType));
  }
}

/* Compute narrow phase collision detection */
void CollisionDetection::runNarrowPhase() {
  /* Swap the pointers for the current and previous contact pairs and manifolds */
  exchangeFrameInfo();
  /* Populate the contacts for each entry in the narrow phase input which includes creating the contact pair and populating the manifold for the pair */
  processNarrowPhase(mNarrowPhase, mCurrentContactPairs, mRawManifolds);
  /* Link the contact pairs to their respective bodies */
  associateContactPairs();
  assert(!mCurrentManifolds->size());
}

/* Exchange frame info */
void CollisionDetection::exchangeFrameInfo() {
  if(mLastContactPairs == &mContactPairsA) {
    mLastContactPairs = &mContactPairsB;
    mLastManifolds = & mManifoldsB;
    mCurrentContactPairs = &mContactPairsA;
    mCurrentManifolds = &mManifoldsA;
  }
  else {
    mLastContactPairs = &mContactPairsA;
    mLastManifolds = &mManifoldsA;
    mCurrentContactPairs = &mContactPairsB;
    mCurrentManifolds = &mManifoldsB;
  }
}

/* Update overlap pairs with overlapping nodes from broad phase */
void CollisionDetection::updateOverlapPairs(const DynamicArray<Pair<int32, int32>>& overlapNodes) {
  const uint32 numOverlapNodes = static_cast<uint32>(overlapNodes.size());

  for(uint32 i = 0; i < numOverlapNodes; i++) {
    Pair<int32, int32> overlapNodePair = overlapNodes[i];
    assert(overlapNodePair.first != -1 && overlapNodePair.second != -1);

    /* Disregard if both the nodes of the overlap pair are the same */
    if(overlapNodePair.first != overlapNodePair.second) {
      const Entity firstColliderEntity = mIdentifierEntityMap[overlapNodePair.first];
      const Entity secondColliderEntity = mIdentifierEntityMap[overlapNodePair.second];
      const uint32 firstColliderIndex = mColliderComponents.getComponentEntityIndex(firstColliderEntity);
      const uint32 secondColliderIndex = mColliderComponents.getComponentEntityIndex(secondColliderEntity);
      const Entity firstBodyEntity = mColliderComponents.mBodyEntities[firstColliderIndex];
      const Entity secondBodyEntity = mColliderComponents.mBodyEntities[secondColliderIndex];

      /* Disregard if the two colliders are from the same body */
      if(firstBodyEntity != secondBodyEntity) {
        const uint32 numActiveColliderComponents = mColliderComponents.getNumEnabledComponents();
        const bool firstBodyEnabled = firstColliderIndex < numActiveColliderComponents;
        const bool secondBodyEnabled = secondColliderIndex < numActiveColliderComponents;
        bool firstBodyStatic = false;
        bool secondBodyStatic = false;

        if(mBodyComponents.containsComponent(firstBodyEntity)) {
          firstBodyStatic = mBodyComponents.mTypes[mBodyComponents.getComponentEntityIndex(firstBodyEntity)] == BodyType::Static;
        }

        if(mBodyComponents.containsComponent(secondBodyEntity)) {
          secondBodyStatic = mBodyComponents.mTypes[mBodyComponents.getComponentEntityIndex(secondBodyEntity)] == BodyType::Static;
        }

        const bool firstBodyActive = firstBodyEnabled && !firstBodyStatic;
        const bool secondBodyActive = secondBodyEnabled && !secondBodyStatic;

        if(firstBodyActive || secondBodyActive) {
          const Pair<Entity, Entity> bodyIndexPair = OverlapPairs::getBodyIndexPair(firstBodyEntity, secondBodyEntity);

          /* Disregard if these two bodies have been marked as being incompatible for collision */
          if(!mIncompatibleCollisionPairs.contains(bodyIndexPair)) {
            /* Get the overlap pair identifier for this specific pair based on the broad phase identifiers of their colliders */
            const uint64 pairIdentifier = getElegantPair(overlapNodePair.first, overlapNodePair.second);
            OverlapPairs::OverlapPair* overlapPair = mOverlapPairs.getOverlapPair(pairIdentifier);

            /* Check whethers an overlap pair corresponding to the obtained pair identifier is already present in broad phase */
            if(!overlapPair) {
              const unsigned short firstCollisionCategory = mColliderComponents.mCollisionCategories[firstColliderIndex];
              const unsigned short secondCollisionCategory = mColliderComponents.mCollisionCategories[secondColliderIndex];
              const unsigned short firstCollisionFilter = mColliderComponents.mCollisionFilters[firstColliderIndex];
              const unsigned short secondCollisionFilter = mColliderComponents.mCollisionFilters[secondColliderIndex];

              /* Debug */
              /* Disregard if the two shapes cannot collide due to filtering */
              if((firstCollisionFilter & secondCollisionCategory) != 0 && (firstCollisionCategory & secondCollisionFilter) != 0) {
                mOverlapPairs.addOverlapPair(firstColliderIndex, secondColliderIndex);
                LOG("Overlap pair created - First Index: " + std::to_string(firstColliderEntity.getIndex()) + ", Second Index: " + std::to_string(secondColliderEntity.getIndex()) + ", Identifier: " + std::to_string(pairIdentifier));
              }
            }
            else {
              /* The colliders of the overlap pair still overlap so no need to test */
              overlapPair->testOverlap = false;
              LOG("Overlap pair already created - First Index: " + std::to_string(firstColliderEntity.getIndex()) + ", Second Index: " + std::to_string(secondColliderEntity.getIndex()));
            }
          }
        }
      }
    }
  }
}

/* Remove overlap pairs that are not overlapping anymore */
void CollisionDetection::removeOverlapPairs() {
  for(uint64 i = 0; i < mOverlapPairs.mPairs.size(); i++) {
    OverlapPairs::OverlapPair& overlapPair = mOverlapPairs.mPairs[i];

    /* Debug */
    /* Query whether there is still a need to test overlap */
    if(overlapPair.testOverlap) {
      /* If yes, test whether they are still overlapping */
      if(mBroadPhase.testShapesOverlap(overlapPair.firstBroadPhaseIdentifier, overlapPair.secondBroadPhaseIdentifier)) {
        overlapPair.testOverlap = false;
      }
      else {
        /* Otherwise, remove overlap pair from broad phase */
        mOverlapPairs.removeOverlapPair(i);
        i--;
        LOG("Removed overlap pair " + std::to_string(i));
      }
    }
  }
}

/* Filter overlap pairs where only a given body is involved */
void CollisionDetection::filterOverlapPairs(Entity bodyEntity, DynamicArray<uint64>& pairs) {
  const uint32 numPairs = static_cast<uint32>(mOverlapPairs.mPairs.size());

  for(uint32 i = 0; i < numPairs; i++) {
    if(mColliderComponents.getBodyEntity(mOverlapPairs.mPairs[i].firstColliderEntity) == bodyEntity ||
       mColliderComponents.getBodyEntity(mOverlapPairs.mPairs[i].secondColliderEntity) == bodyEntity) {
      pairs.add(mOverlapPairs.mPairs[i].pairIdentifier);
    }
  }
}

/* Filter overlap pairs where two given bodies are involved */
void CollisionDetection::filterOverlapPairs(Entity firstBodyEntity, Entity secondBodyEntity, DynamicArray<uint64>& pairs) {
  const uint32 numPairs = static_cast<uint32>(mOverlapPairs.mPairs.size());

  for(uint32 i = 0; i < numPairs; i++) {
    const Entity firstColliderBodyEntity = mColliderComponents.getBodyEntity(mOverlapPairs.mPairs[i].firstColliderEntity);
    const Entity secondColliderBodyEntity = mColliderComponents.getBodyEntity(mOverlapPairs.mPairs[i].secondColliderEntity);

    if((firstColliderBodyEntity == firstBodyEntity && secondColliderBodyEntity == secondBodyEntity) ||
       (firstColliderBodyEntity == secondBodyEntity && secondColliderBodyEntity == firstBodyEntity)) {
      pairs.add(mOverlapPairs.mPairs[i].pairIdentifier);
    }
  }
}

/* Process narrow phase input */
void CollisionDetection::processNarrowPhase(NarrowPhase& narrowPhase, DynamicArray<ContactPair>* contactPairs, DynamicArray<LocalManifold>& manifolds) {
  assert(!contactPairs->size());
  const uint32 numNarrowPhaseEntries = static_cast<uint32>(narrowPhase.entries.size());
  LOG("Retrieved " + std::to_string(numNarrowPhaseEntries) + " narrow phase entry(s)");

  for(uint32 i = 0; i < numNarrowPhaseEntries; i++) {
    LocalManifoldInfo manifoldInfo;
    /* Debug */
    /* Execute the appropriate narrow phase algorithm based on the types of the collision shapes */
    narrowPhase.entries[i].algorithm->execute(narrowPhase, i, manifoldInfo);

    /* Only process the result into a contact pair if the two shape have been found to be colliding (there is more than one contact point)*/
    if(narrowPhase.entries[i].isColliding) {
      LOG("Collision detected for narrow phase entry " + std::to_string(i) + " associated with overlap pair identifier " + std::to_string(narrowPhase.entries[i].overlapPairIdentifier));
      const uint64 pairIdentifier = narrowPhase.entries[i].overlapPairIdentifier;
      OverlapPairs::OverlapPair* overlapPair = mOverlapPairs.getOverlapPair(pairIdentifier);
      assert(overlapPair);
      const Entity firstColliderEntity = narrowPhase.entries[i].firstColliderEntity;
      const Entity secondColliderEntity = narrowPhase.entries[i].secondColliderEntity;
      const uint32 firstColliderIndex = mColliderComponents.getComponentEntityIndex(firstColliderEntity);
      const uint32 secondColliderIndex = mColliderComponents.getComponentEntityIndex(secondColliderEntity);
      const Entity firstBodyEntity = mColliderComponents.mBodyEntities[firstColliderIndex];
      const Entity secondBodyEntity = mColliderComponents.mBodyEntities[secondColliderIndex];
      /* Confirm that neither of the bodies involved in the contact is not disabled */
      assert(!mWorld->mBodyComponents.getIsEntityDisabled(firstBodyEntity) || !mWorld->mBodyComponents.getIsEntityDisabled(secondBodyEntity));
      const uint32 newContactPairIndex = static_cast<uint32>(contactPairs->size());
      /* Add the contact pair to the array */
      LOG("Creating contact pair for - First Index: " + std::to_string(firstColliderEntity.getIndex()) + ", Second Index: " + std::to_string(secondColliderEntity.getIndex()));
      contactPairs->emplace(pairIdentifier, newContactPairIndex, firstBodyEntity, secondBodyEntity, firstColliderEntity, secondColliderEntity);
      const uint32 newManifoldIndex = static_cast<uint32>(manifolds.size());
      /* Add the manifold for the contact pair into the array */
      manifolds.emplace(manifoldInfo, firstBodyEntity, secondBodyEntity, firstColliderEntity, secondColliderEntity);
      /* Associate this manifold with the contact pair */
      (*contactPairs)[newContactPairIndex].rawManifoldsIndex = newManifoldIndex;
    }
  }
}

/* Add the contact pairs to the appropriate bodies */
void CollisionDetection::associateContactPairs() {
  std::stringstream ss;
  const uint32 numCurrentContactPairs = static_cast<uint32>(mCurrentContactPairs->size());

  /* Add the contact pairs to both bodies of the pair so that we can create islands for contact solving */
  for(uint32 i = 0; i < numCurrentContactPairs; i++) {
    ContactPair& contactPair = (*mCurrentContactPairs)[i];
    mBodyComponents.addContactPair(contactPair.firstBodyEntity, i);
    mBodyComponents.addContactPair(contactPair.secondBodyEntity, i);
    ss << "Added contact pair " << i << " to bodies - First Index: " << contactPair.firstBodyEntity.getIndex() << ", Second Index: " << contactPair.secondBodyEntity.getIndex();
  }
}

/* Prepare collision detection results for the contact solver */
void CollisionDetection::prepareForContactSolver() {
  std::stringstream ss;
  mCurrentManifolds->reserve(mCurrentContactPairs->size());
  const uint32 numContactPairs = static_cast<uint32>(mWorld->mIslandOrderedContactPairs.size());

  /* Process the pairs in the order they assumed during island creation */
  for(uint32 i = 0; i < numContactPairs; i++) {
    uint32 contactPairIndex = mWorld->mIslandOrderedContactPairs[i];
    ContactPair& contactPair = (*mCurrentContactPairs)[contactPairIndex];
    contactPair.manifoldsIndex = static_cast<uint32>(mCurrentManifolds->size());
    mCurrentManifolds->add(mRawManifolds[contactPair.rawManifoldsIndex]);
  }

  ss << "Current manifold count is " << mCurrentManifolds->size() << std::endl;

  /* Copy the impulses from the contact points of the manifolds in the previous frame */
  prepareForWarmStart();
  mLastContactPairs->clear();
  mLastManifolds->clear(true);
  /* Map overlap pair to the contact pair index so that we can copy impulses for warm starting */
  populateLastContactPairMap();
  mNarrowPhase.clear();
}

/* Update the current contact information with those of the last frame for warm starting */
void CollisionDetection::prepareForWarmStart() {
  const uint32 numContactPairs = static_cast<uint32>(mCurrentContactPairs->size());

  /* Match old contact identifers to new contact identifiers and copy the stored impulses so that we can warm start the contact solver */
  for(uint32 i = 0; i < numContactPairs; i++) {
    ContactPair& currentContactPair = (*mCurrentContactPairs)[i];
    /* Map the common overlap pair identifier to the corresponding contact pair in the previous frame */
    auto iterLastContactPair = mOverlapPairLastContactPairMap.find(currentContactPair.overlapPairIdentifier);

    /* If a contact pair existed in the previous frame, copy the information from the old manifold to the new manifold */
    if(iterLastContactPair != mOverlapPairLastContactPairMap.end()) {
      const uint32 lastContactPairIndex = iterLastContactPair->second;
      ContactPair& lastContactPair = (*mLastContactPairs)[lastContactPairIndex];

      for(uint8 j = 0; j < (*mCurrentManifolds)[currentContactPair.manifoldsIndex].info.numPoints; j++) {
        ContactPoint* currentPoint = (*mCurrentManifolds)[currentContactPair.manifoldsIndex].info.points + j;
        currentPoint->normalImpulse = 0.0f;
        currentPoint->tangentImpulse = 0.0f;
        ContactInfo info = currentPoint->info;

        for(uint8 k = 0; k < (*mLastManifolds)[lastContactPair.manifoldsIndex].info.numPoints; k++) {
          ContactPoint* lastPoint = (*mLastManifolds)[lastContactPair.manifoldsIndex].info.points + k;

          if(lastPoint->info.key == info.key) {
            currentPoint->normalImpulse = lastPoint->normalImpulse;
            currentPoint->tangentImpulse = lastPoint->tangentImpulse;
            break;
          }
        }
      }
    }
  }
}

/* Populate the overlap pair identifier to last frame contact pair index map */
void CollisionDetection::populateLastContactPairMap() {
  mOverlapPairLastContactPairMap.clear();
  const uint32 numContactPairs = static_cast<uint32>(mCurrentContactPairs->size());

  for(uint32 i = 0; i < numContactPairs; i++) {
    mOverlapPairLastContactPairMap.insert(Pair<uint64, uint32>((*mCurrentContactPairs)[i].overlapPairIdentifier, i));
  }
}

/* Execute collision detection */
void CollisionDetection::execute() {
  /* Execute broad phase collision detection */
  runBroadPhase();
  /* Prepare for narrow phase collision detection */
  prepareNarrowPhase(mNarrowPhase);
  /* Execute narrow phase collision detection */
  runNarrowPhase();
}

/* Add collider to the collision detection system */
void CollisionDetection::addCollider(Collider* collider, const AABB& aabb) {
  /* Insert the collider into the dynamic tree */
  mBroadPhase.addCollider(collider, aabb);
  int32 broadPhaseIdentifier = mColliderComponents.getBroadPhaseIdentifier(collider->getEntity());
  assert(!mIdentifierEntityMap.contains(broadPhaseIdentifier));
  /* Map the broad phase identifier of the collider to its entity */
  mIdentifierEntityMap.insert(Pair<int32, Entity>(broadPhaseIdentifier, collider->getEntity()));
}

/* Remove collider from the collision detection system */
void CollisionDetection::removeCollider(Collider* collider) {
  const int32 broadPhaseIdentifier = collider->getBroadPhaseIdentifier();
  assert(broadPhaseIdentifier != -1);
  assert(mIdentifierEntityMap.contains(broadPhaseIdentifier));
  DynamicArray<uint64>& overlapPairs = mColliderComponents.getOverlapPairs(collider->getEntity());

  /* If we remove the collider, we need to remove all of the overlap pairs in broad phase in which this collider is involved */
  while(!overlapPairs.empty()) {
    mOverlapPairs.eraseOverlapPair(overlapPairs[0]);
  }

  mIdentifierEntityMap.remove(broadPhaseIdentifier);
  /* Remove the collider from the dynamic tree */
  mBroadPhase.removeCollider(collider);
}

/* Update a collider in the collision detection system */
void CollisionDetection::updateCollider(Entity entity) {
  mBroadPhase.updateCollider(entity);
}

/* Update all colliders in the collision detection system */
void CollisionDetection::updateColliders() {
  mBroadPhase.updateColliders();
}

/* Add body pair that are incompatible for collision */
void CollisionDetection::addIncompatibleCollisionPair(Entity firstBodyEntity, Entity secondBodyEntity) {
  mIncompatibleCollisionPairs.insert(OverlapPairs::getBodyIndexPair(firstBodyEntity, secondBodyEntity));

  /* Since we are adding an incompatible collision pair, we need to ensure that we remove the associated overlap pairs since their colliders should not be colliding anymore */
  DynamicArray<uint64> removalPairs(mMemoryStrategy.getObjectPoolMemoryHandler());
  const uint32 numRemovalPairs = static_cast<uint32>(removalPairs.size());
  /* Colliders of the first body */
  const DynamicArray<Entity>& colliderEntities = mWorld->mBodyComponents.getColliders(firstBodyEntity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    /* Overlap pairs of the first body */
    const DynamicArray<uint64>& overlapPairs = mColliderComponents.getOverlapPairs(colliderEntities[i]);
    const uint32 numOverlapPairs = static_cast<uint32>(overlapPairs.size());

    for(uint32 j = 0; j < numOverlapPairs; j++) {
      OverlapPairs::OverlapPair* overlapPair = mOverlapPairs.getOverlapPair(overlapPairs[j]);
      assert(overlapPair);
      const Entity firstOverlapBody = mOverlapPairs.mColliderComponents.getBodyEntity(overlapPair->firstColliderEntity);
      const Entity secondOverlapBody = mOverlapPairs.mColliderComponents.getBodyEntity(overlapPair->secondColliderEntity);

      /* Check if the second body is involved in the current overlap pair of the first body */
      if(firstOverlapBody == secondBodyEntity || secondOverlapBody == secondBodyEntity) {
        removalPairs.add(overlapPairs[j]);
      }
    }
  }

  for(uint32 i = 0; i < numRemovalPairs; i++) {
      mOverlapPairs.eraseOverlapPair(removalPairs[i]);
  }
}

/* Remove body pair that are incompatible for collision */
void CollisionDetection::removeIncompatibleCollisionPair(Entity firstBodyEntity, Entity secondBodyEntity) {
  mIncompatibleCollisionPairs.remove(OverlapPairs::getBodyIndexPair(firstBodyEntity, secondBodyEntity));
}

/* Forcibly test shape for broad phase collision */
void CollisionDetection::checkBroadPhaseCollision(Collider* collider) {
  if(collider->getBroadPhaseIdentifier() != -1) {
     mBroadPhase.addColliderForTest(collider->getBroadPhaseIdentifier(), collider);
  }
}

/* Notify that overlap pairs where the given collider is involved need to be tested for overlap */
void CollisionDetection::notifyOverlapPairsToTest(Collider* collider) {
  /* Overlap pairs of the given collider */
  DynamicArray<uint64>& overlapPairs = mColliderComponents.getOverlapPairs(collider->getEntity());
  const uint32 numOverlapPairs = static_cast<uint32>(overlapPairs.size());

  for(uint32 i = 0; i < numOverlapPairs; i++) {
    mOverlapPairs.setTestOverlap(overlapPairs[i], true);
  }
}

/* Get collision algorithm dispatch */
AlgorithmDispatch& CollisionDetection::getAlgorithmDispatch() {
  return mAlgorithmDispatch;
}

/* Return a reference to memory strategy */
MemoryStrategy& CollisionDetection::getMemoryStrategy() const {
  return mMemoryStrategy;
}