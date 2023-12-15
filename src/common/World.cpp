#include <physics/common/Factory.h>
#include <physics/common/World.h>
#include <physics/collections/DynamicArray.h>
#include <physics/collections/Stack.h>

using namespace physics;

/* Constructor */
World::World(MemoryStrategy& memoryStrategy,
             Factory& factory, 
             const Settings& settings) :
             mMemoryStrategy(memoryStrategy),
             mSettings(settings),
             mEntityHandler(mMemoryStrategy.getFreeListMemoryHandler()),
             mBodyComponents(mMemoryStrategy.getFreeListMemoryHandler()),
             mColliderComponents(mMemoryStrategy.getFreeListMemoryHandler()),
             mTransformComponents(mMemoryStrategy.getFreeListMemoryHandler()),
             mCollisionDetection(this,
                                 mMemoryStrategy, 
                                 mBodyComponents,
                                 mColliderComponents,
                                 mTransformComponents),
             mBodies(mMemoryStrategy.getFreeListMemoryHandler()),
             mIslands(mMemoryStrategy.getLinearMemoryHandler()),
             mIslandOrderedContactPairs(mMemoryStrategy.getLinearMemoryHandler()),
             mContactSolver(*this,
                            mMemoryStrategy,
                            mIslands,
                            mBodyComponents,
                            mColliderComponents,
                            mTransformComponents,
                            mSettings.restitutionThreshold),
            mIsGravityEnabled(true),
            mDynamics(*this,
                      mBodyComponents,
                      mColliderComponents,
                      mTransformComponents,
                      mIsGravityEnabled,
                      mSettings.gravity),
            mNumVelocitySolverIterations(mSettings.defaultVelocityConstraintSolverIterations),
            mNumPositionSolverIterations(mSettings.defaultPositionConstraintSolverIterations),
            mIsSleepingEnabled(mSettings.isSleepingEnabled),
            mSleepLinearVelocity(mSettings.defaultLinearVelocityForSleep),
            mSleepAngularSpeed(mSettings.defaultAngularSpeedForSleep),
            mSleepTime(mSettings.defaultSleepTime),
            mLastInverseDelta(0.0f) {}

/* Destructor */
World::~World() {
  uint32 i = static_cast<uint32>(mBodies.size());

  /* Destroy bodies which still exist in the world */
  while(i) {
    i--;
    destroyBody(mBodies[i]);
  }

  assert(!mBodies.size());
  assert(!mBodyComponents.getNumComponents());
  assert(!mColliderComponents.getNumComponents());
  assert(!mTransformComponents.getNumComponents());
}

/* Set a body to be disabled */
void World::disableBody(Entity entity, bool isDisabled) {
  if(isDisabled == mBodyComponents.getIsEntityDisabled(entity)) {
    return;
  }

  assert(mBodyComponents.containsComponent(entity));
  /* Notify components */
  mBodyComponents.setIsEntityDisabled(entity, isDisabled);
  mTransformComponents.setIsEntityDisabled(entity, isDisabled);
  
  /* For each collider notify the component that its owning body has become disabled */
  const DynamicArray<Entity>& colliderEntities = mBodyComponents.getColliders(entity);
  const uint32 numColliderEntities = static_cast<uint32>(colliderEntities.size());

  for(uint32 i = 0; i < numColliderEntities; i++) {
    mColliderComponents.setIsEntityDisabled(colliderEntities[i], isDisabled);
  }
} 

/* Generate the islands of the current frame */
void World::generateIslands() {
  assert(mIslandOrderedContactPairs.size() == 0);
  const uint32 numBodyComponents = mBodyComponents.getNumComponents();

  /* Reset island inclusion state */
  for(uint32 i = 0; i < numBodyComponents; i++) {
    mBodyComponents.mIsInIsland[i] = false;
  }

  /* Reserve memory based on capacity from the previous frame */
  mIslands.reserve();
  /* Bodies to visit during DFS */
  Stack<Entity> visitBodies(mMemoryStrategy.getLinearMemoryHandler());
  /* Keep track of static bodies in the current island */
  DynamicArray<Entity> visitedStaticBodies(mMemoryStrategy.getLinearMemoryHandler(), 16);
  uint32 numManifolds = 0;

  /* Only form islands with enabled bodies */
  for(uint32 i = 0; i < mBodyComponents.getNumEnabledComponents(); i++) {
    /* Disregard if body has already been added to an island or the body is static */
    if(mBodyComponents.mIsInIsland[i] ||
       mBodyComponents.mTypes[i] == BodyType::Static) {
      continue;
    }

    /* Reset stack of bodies to visit during DFS */
    visitBodies.clear();
    mBodyComponents.mIsInIsland[i] = true;
    /* Add body into the stack of bodies to visit during DFS */
    visitBodies.push(mBodyComponents.mBodyEntities[i]);
    /* Create a new island */
    uint32 islandIndex = mIslands.addIsland(numManifolds);

    /* There are still bodies to visit */
    while(!visitBodies.empty()) {
      /* Get entity of visited body */
      const Entity visitedBody = visitBodies.pop();
      /* Add the body into the island */
      mIslands.addBody(visitedBody);
      /* Wake up the body in the case that it is sleeping */
      mBodyComponents.getBody(visitedBody)->setIsSleeping(false);
      /* If the body was awaken, we need to get its index in the body components array */
      const uint32 visitedBodyIndex = mBodyComponents.getComponentEntityIndex(visitedBody);

      /* If static body, disregard in DFS and continue to the next body  */
      if(mBodyComponents.mTypes[visitedBodyIndex] == BodyType::Static) {
        visitedStaticBodies.add(visitedBody);
        continue;
      }

      const uint32 numContactPairs = static_cast<uint32>(mBodyComponents.mContactPairs[visitedBodyIndex].size());
      LOG("Island generation found " + std::to_string(numContactPairs) + " contact pair(s) for body index " + std::to_string(visitedBody.getIndex()));

      /* Check the other contact pairs that the current body is involved in */
      for(uint32 j = 0; j < numContactPairs; j++) {
        const uint32 contactPairIndex = mBodyComponents.mContactPairs[visitedBodyIndex][j];
        ContactPair& contactPair = (*mCollisionDetection.mCurrentContactPairs)[contactPairIndex];
        
        /* Disregard if the current contact pair has been added into an island */
        if(contactPair.isInIsland) {
          continue;
        }

        const Entity oppositeBody = contactPair.firstBodyEntity == visitedBody ? contactPair.secondBodyEntity : contactPair.firstBodyEntity;
        LOG("Opposite body index " + std::to_string(oppositeBody.getIndex()) + " found for parent body index " + std::to_string(visitedBody.getIndex()));

        if(mBodyComponents.containsComponent(oppositeBody)) {
          uint32 oppositeBodyIndex = mBodyComponents.getComponentEntityIndex(oppositeBody);
          mIslandOrderedContactPairs.add(contactPairIndex);
          numManifolds++;
          /* Add the contact manifold into the current island */
          mIslands.numManifolds[islandIndex]++;
          contactPair.isInIsland = true;

          /* Disregard visiting the body is it has already been added to an island */
          if(mBodyComponents.mIsInIsland[oppositeBodyIndex]) {
            continue;
          }

          /* Otherwise, add to the stack of bodies to visit */
          visitBodies.push(oppositeBody);
          mBodyComponents.mIsInIsland[oppositeBodyIndex] = true;
        }
        else {
          /* Otherwise, add the contact pair into the array of contact pairs that aren't included in the islands */
          contactPair.isInIsland = true;
        }
      }
    }

    const uint32 numVisitedStaticBodies = static_cast<uint32>(visitedStaticBodies.size());

    /* Allow the static bodies to be included in other islands */
    for(uint32 j = 0; j < numVisitedStaticBodies; j++) {
      assert(mBodyComponents.getType(visitedStaticBodies[j]) == BodyType::Static);
      mBodyComponents.setIsInIsland(visitedStaticBodies[j], false);
    }

    visitedStaticBodies.clear();
  }

  const uint32 numEnabledBodyComponents = mBodyComponents.getNumEnabledComponents();

  /* Clear contact pairs linked to bodies */
  for(uint32 i = 0; i < numEnabledBodyComponents; i++) {
    mBodyComponents.mContactPairs[i].clear();
  }
}

/* Solve the physics simulation */
void World::solve(TimeStep timeStep) {
  /* Initialize constrained positions and orientations */
  mDynamics.initializeStateConstraints();

  /* Integrate the linear and angular velocities using forces and torques */
  mDynamics.integrateVelocities(timeStep);

  /* Initialize the contact solver for each island */
  mContactSolver.initialize(mCollisionDetection.mCurrentManifolds, timeStep);

  /* Solve velocity constraints */
  for(uint32 i = 0; i < mNumVelocitySolverIterations; i++) {
    mContactSolver.solveVelocityConstraints();
  }

  /* Store impulses for warm starting */
  mContactSolver.storeImpulses();

  /* Integrate positions using the constrained velocities */
  mDynamics.integratePositions(timeStep);

  /* Solve position constraints */
  for(uint32 i = 0; i < mNumPositionSolverIterations; i++) {
    mContactSolver.solvePositionConstraints();
  }

  /* Reset the contact solver */
  mContactSolver.reset();
}

/* Set bodies to sleep as appropriate */
void World::sleepBodies(TimeStep timeStep) {
  const uint32 numIslands = mIslands.getNumIslands();

  /* For each island */
  for(uint32 i = 0; i < numIslands; i++) {
    float minSleepTime = FLOAT_LARGEST;

    for(uint32 j = 0; j < mIslands.numBodies[i]; j++) {
      const Entity entity = mIslands.bodies[mIslands.bodyIndices[i] + j];
      const uint32 index = mBodyComponents.getComponentEntityIndex(entity);

      /* Disregard static bodies */
      if(mBodyComponents.mTypes[index] == BodyType::Static) {
        continue;
      }

      /* Velocity is large enough to stay awake */
      if(mBodyComponents.mLinearVelocities[index].lengthSquare() > square(mSleepLinearVelocity) ||
         square(mBodyComponents.mAngularSpeeds[index]) > square(mSleepAngularSpeed) ||
         !mBodyComponents.mIsAllowedToSleep[index]) {
        /* Reset sleep time */
        mBodyComponents.mSleepTimes[index] = 0.0f;
        minSleepTime = 0.0f;   
      }
      /* Velocity is below the threshold */
      else {
        /* Increase sleep time */
        mBodyComponents.mSleepTimes[index] += timeStep.delta;

        if(mBodyComponents.mSleepTimes[index] < minSleepTime) {
          minSleepTime = mBodyComponents.mSleepTimes[index];
        }
      }
    }

    /* Velocity of all bodies is under the threshold for longer than the sleep time */
    if(minSleepTime >= mSleepTime && mIslands.solved[i]) {
      for(uint32 j = 0; j < mIslands.numBodies[i]; j++) {
        const Entity entity = mIslands.bodies[mIslands.bodyIndices[i] + j];
        Body* body = mBodyComponents.getBody(entity);
        body->setIsSleeping(true);
      }
    }
  }
}

/* Update the physics simulation */
void World::step(float dt) {
  TimeStep timeStep;
  timeStep.delta = dt;
  timeStep.inverseDelta = dt > 0.0f ? 1.0f / dt : 0.0f;
  timeStep.deltaRatio = mLastInverseDelta * dt;
  
  /* Execute collision detection */
  mCollisionDetection.execute();
  /* Create the islands */
  generateIslands();
  /* Prepare the collision detection results for the contact solver */
  mCollisionDetection.prepareForContactSolver();
  /* Compute the parameters of the simulation  */
  solve(timeStep);
  /* Update the actual positions and velocities of the bodies */
  mDynamics.updateBodyStates();
  /* Update collider components */
  mCollisionDetection.updateColliders();

  /* Update sleeping bodies */
  if(mIsSleepingEnabled) {
    sleepBodies(timeStep);
  }

  /* Reset external forces and torques */
  mDynamics.resetExternalStimuli();
  /* Reset islands */
  mIslands.clear();
  /* Clear ordered contact pairs */
  mIslandOrderedContactPairs.clear(true);
  /* Reset frame memory */
  mMemoryStrategy.reset(MemoryStrategy::HandlerType::Linear);
}

/* Create a body */
Body* World::createBody(const Transform& transform) {
  /* New entity for the body */
  Entity entity = mEntityHandler.createEntity();
  /* New transform for the body */
  mTransformComponents.insertComponent(entity, false, TransformComponents::TransformComponent(transform));
  /* Create the actual body */
  Body* body = new (mMemoryStrategy.allocate(MemoryStrategy::HandlerType::ObjectPool, sizeof(Body))) Body(*this, entity);
  assert(body);
  /* Create a component for the body in the components array */
  BodyComponents::BodyComponent bodyComponent(body, BodyType::Dynamic, transform.getPosition());
  mBodyComponents.insertComponent(entity, false, bodyComponent);
  /* Compute the inverse mass */
  mBodyComponents.setInverseMass(entity, 1.0f / mBodyComponents.getMass(entity));
  /* Add the body to the world */
  mBodies.add(body);

  LOG("Created body with entity index " + std::to_string(body->getEntity().getIndex()));

  return body;
}

/* Destroy a body */
void World::destroyBody(Body* body) {
  LOG("Removing body with entity index " + std::to_string(body->getEntity().getIndex()));
  /* Remove all colliders associated with the body */
  body->removeColliders();
  /* Remove the component for the body from the components array */
  mBodyComponents.removeComponent(body->getEntity());
  /* Remove the transform for the body */
  mTransformComponents.removeComponent(body->getEntity());
  /* Destroy the body's entity */
  mEntityHandler.deleteEntity(body->getEntity());
  /* Destroy the body itself */
  body->~Body();
  /* Remove the body from the world */
  mBodies.remove(body);
  /* Release the memory that the body was occupying */
  mMemoryStrategy.free(MemoryStrategy::HandlerType::ObjectPool, body, sizeof(Body));
}

