#include <physics/dynamics/ContactSolver.h>
#include <physics/common/World.h>
#include <physics/dynamics/Body.h>
#include <physics/collision/Collider.h>
#include <physics/dynamics/Islands.h>
#include <physics/common/Factory.h>

using namespace physics;

/* Constructor */
ContactSolver::ContactSolver(World& world,
                             MemoryStrategy& memoryStrategy,
                             Islands& islands,
                             BodyComponents& bodyComponents,
                             ColliderComponents& colliderComponents,
                             TransformComponents& transformComponents,
                             float& restitutionThreshold) :
                             mWorld(world),
                             mMemoryStrategy(memoryStrategy),
                             mIslands(islands),
                             mManifolds(nullptr),
                             mBodyComponents(bodyComponents),
                             mColliderComponents(colliderComponents),
                             mTransformComponents(transformComponents),
                             mRestitutionThreshold(restitutionThreshold) {}

/* Compute the collision restituion factor */
float ContactSolver::computeMixedRestitution(const Material& firstMaterial, const Material& secondMaterial) const {
  const float firstRestitution = firstMaterial.getRestitution();
  const float secondRestitution = secondMaterial.getRestitution();
  return (firstRestitution > secondRestitution) ? firstRestitution : secondRestitution;
}

/* Compute the mixed friction coefficient */
float ContactSolver::computeMixedFriction(const Material& firstMaterial, const Material& secondMaterial) const {
  return std::sqrt(firstMaterial.getFriction() * secondMaterial.getFriction());
}

/* Initialize contact solver for a given island */
void ContactSolver::initializeIsland(uint32 islandIndex) {
  assert(mIslands.numBodies[islandIndex]);
  assert(mIslands.numManifolds[islandIndex]);
  const uint32 manifoldIndex = mIslands.manifoldIndices[islandIndex];
  const uint32 numManifolds = mIslands.numManifolds[islandIndex];

  for (uint32 i = manifoldIndex; i < manifoldIndex + numManifolds; i++) {
    LocalManifold& manifold = (*mManifolds)[i];
    assert(manifold.info.numPoints);
    const uint32 firstBodyIndex = mBodyComponents.getComponentEntityIndex(manifold.firstBodyEntity);
    const uint32 secondBodyIndex = mBodyComponents.getComponentEntityIndex(manifold.secondBodyEntity);
    assert(!mBodyComponents.getIsEntityDisabled(manifold.firstBodyEntity));
    assert(!mBodyComponents.getIsEntityDisabled(manifold.secondBodyEntity));
    const uint32 firstColliderIndex = mColliderComponents.getComponentEntityIndex(manifold.firstColliderEntity);
    const uint32 secondColliderIndex = mColliderComponents.getComponentEntityIndex(manifold.secondColliderEntity);

    VelocityConstraint* velocityConstraint = mVelocityConstraints + i;
    velocityConstraint->friction = computeMixedFriction(mColliderComponents.mMaterials[firstColliderIndex], mColliderComponents.mMaterials[secondColliderIndex]);
    velocityConstraint->restitution = computeMixedRestitution(mColliderComponents.mMaterials[firstColliderIndex], mColliderComponents.mMaterials[secondColliderIndex]);
    velocityConstraint->inverseMassA = mBodyComponents.mInverseMasses[firstBodyIndex];
    velocityConstraint->inverseMassB = mBodyComponents.mInverseMasses[secondBodyIndex];
    velocityConstraint->inverseInertiaA = mBodyComponents.mInverseInertias[firstBodyIndex];
    velocityConstraint->inverseInertiaB = mBodyComponents.mInverseInertias[secondBodyIndex];
    velocityConstraint->numPoints = manifold.info.numPoints;
    velocityConstraint->K.setZero();
    velocityConstraint->normalMass.setZero();

    PositionConstraint* positionConstraint = mPositionConstraints + i;
    positionConstraint->inverseMassA = mBodyComponents.mInverseMasses[firstBodyIndex];
    positionConstraint->inverseMassB = mBodyComponents.mInverseMasses[secondBodyIndex];
    positionConstraint->inverseInertiaA = mBodyComponents.mInverseInertias[firstBodyIndex];
    positionConstraint->inverseInertiaB = mBodyComponents.mInverseInertias[secondBodyIndex];
    positionConstraint->localCenterA = mBodyComponents.mCentersOfMassLocal[firstBodyIndex];
    positionConstraint->localCenterB = mBodyComponents.mCentersOfMassLocal[secondBodyIndex];
    positionConstraint->radiusA = mColliderComponents.mShapes[firstColliderIndex]->getRadius();
    positionConstraint->radiusB = mColliderComponents.mShapes[secondColliderIndex]->getRadius();
    positionConstraint->localNormal = manifold.info.localNormal;
    positionConstraint->localPoint = manifold.info.localPoint;
    positionConstraint->numPoints = manifold.info.numPoints;
    positionConstraint->type = manifold.info.type;

    uint32 numPoints = manifold.info.numPoints;

    for(uint32 j = 0; j < numPoints; j++) {
      ContactPoint* contactPoint = manifold.info.points + j;
      VelocityConstraint::VelocityConstraintPoint* constraintPoint = velocityConstraint->points + j;
      constraintPoint->normalImpulse = mTimeStep.deltaRatio * contactPoint->normalImpulse;
      constraintPoint->tangentImpulse = mTimeStep.deltaRatio * contactPoint->tangentImpulse;
      constraintPoint->rA.setZero();
      constraintPoint->rB.setZero();
      constraintPoint->normalMass = 0.0f;
      constraintPoint->tangentMass = 0.0f;
      constraintPoint->velocityBias = 0.0f;
      positionConstraint->points[j] = contactPoint->localPoint;
    }

    mNumManifolds++;
  }
}

/* Initialize velocity constraints */
void ContactSolver::initializeVelocityConstraints() {
  for (uint32 i = 0; i < mNumManifolds; i++) {
    LocalManifold& localManifold = (*mManifolds)[i];
    assert(localManifold.info.numPoints);

    const uint32 firstBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.firstBodyEntity);
    const uint32 secondBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.secondBodyEntity);

    VelocityConstraint* velocityConstraint = mVelocityConstraints + i;
    PositionConstraint* positionConstraint = mPositionConstraints + i;

    float radiusA = positionConstraint->radiusA;
    float radiusB = positionConstraint->radiusB;
    float inverseMassA = velocityConstraint->inverseMassA;
    float inverseMassB = velocityConstraint->inverseMassB;
    float inverseInertiaA = velocityConstraint->inverseInertiaA;
    float inverseInertiaB = velocityConstraint->inverseInertiaB;
    Vector2 localCenterA = positionConstraint->localCenterA;
    Vector2 localCenterB = positionConstraint->localCenterB;
    
    Vector2 positionA = mBodyComponents.mPositionsConstrained[firstBodyIndex];
    float angleA = mBodyComponents.mOrientationsConstrained[firstBodyIndex].getAngle();
    Vector2 linearVelocityA = mBodyComponents.mLinearVelocitiesConstrained[firstBodyIndex];
    float angularSpeedA = mBodyComponents.mAngularSpeedsConstrained[firstBodyIndex];

    Vector2 positionB = mBodyComponents.mPositionsConstrained[secondBodyIndex];
    float angleB = mBodyComponents.mOrientationsConstrained[secondBodyIndex].getAngle();
    Vector2 linearVelocityB = mBodyComponents.mLinearVelocitiesConstrained[secondBodyIndex];
    float angularSpeedB = mBodyComponents.mAngularSpeedsConstrained[secondBodyIndex];

    Transform transformA;
    Transform transformB;
    transformA.setOrientation(Rotation(angleA));
    transformB.setOrientation(Rotation(angleB));
    transformA.setPosition(positionA - (transformA.getOrientation() * localCenterA));
    transformB.setPosition(positionB - (transformB.getOrientation() * localCenterB));

    WorldManifold worldManifold(localManifold, transformA, radiusA, transformB, radiusB);
    velocityConstraint->normal = worldManifold.normal;
    uint32 numPoints = velocityConstraint->numPoints;

    for(uint32 j = 0; j < numPoints; j++) {
      VelocityConstraint::VelocityConstraintPoint* constraintPoint = velocityConstraint->points + j;
      constraintPoint->rA = worldManifold.points[j] - positionA;
      constraintPoint->rB = worldManifold.points[j] - positionB;

      float rnA = cross(constraintPoint->rA, velocityConstraint->normal);
      float rnB = cross(constraintPoint->rB, velocityConstraint->normal);

      float kN = inverseMassA + inverseMassB + inverseInertiaA * rnA * rnA + inverseInertiaB * rnB * rnB;
      constraintPoint->normalMass = kN > 0.0f ? 1.0f / kN : 0.0f;
      Vector2 tangent = cross(velocityConstraint->normal, 1.0f);
      float rtA = cross(constraintPoint->rA, tangent);
      float rtB = cross(constraintPoint->rB, tangent);
      float kT = inverseMassA + inverseMassB + inverseInertiaA * rtA * rtA + inverseInertiaB * rtB * rtB;
      constraintPoint->tangentMass = kT > 0.0f ? 1.0f / kT : 0.0f;
      constraintPoint->velocityBias = 0.0f;
      float relativeVelocity = dot(velocityConstraint->normal, linearVelocityB + cross(angularSpeedB, constraintPoint->rB) - linearVelocityA - cross(angularSpeedA, constraintPoint->rA));

      /* Debug */
      /* (-) */
      if(relativeVelocity < -mRestitutionThreshold) {
        constraintPoint->velocityBias = -velocityConstraint->restitution * relativeVelocity;
      }
    }

    if(velocityConstraint->numPoints == MAX_MANIFOLD_POINTS) {
      VelocityConstraint::VelocityConstraintPoint* constraintPointA = velocityConstraint->points + 0;
      VelocityConstraint::VelocityConstraintPoint* constraintPointB = velocityConstraint->points + 1;
      float rn1A = cross(constraintPointA->rA, velocityConstraint->normal);
      float rn1B = cross(constraintPointA->rB, velocityConstraint->normal);
      float rn2A = cross(constraintPointB->rA, velocityConstraint->normal);
      float rn2B = cross(constraintPointB->rB, velocityConstraint->normal);
      float k11 = inverseMassA + inverseMassB + inverseInertiaA * rn1A * rn1A + inverseInertiaB * rn1B * rn1B;
      float k22 = inverseMassA + inverseMassB + inverseInertiaA * rn2A * rn2A + inverseInertiaB * rn2B * rn2B;
      float k12 = inverseMassA + inverseMassB + inverseInertiaA * rn1A * rn2A + inverseInertiaB * rn1B * rn2B;

      if(square(k11) < BLOCK_SOLVER_MAX_CONDITION * (k11 * k22 - square(k12))) {
        velocityConstraint->K.set({k11, k12}, {k12, k22});
        velocityConstraint->normalMass = velocityConstraint->K.getInverse();
      }
      else {
        velocityConstraint->numPoints = 1;
      }
    }
  }
}

/* Warm start the solver */
void ContactSolver::warmStart() {
  for (uint32 i = 0; i < mNumManifolds; i++) {
    LocalManifold& localManifold = (*mManifolds)[i];
    assert(localManifold.info.numPoints);

    const uint32 firstBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.firstBodyEntity);
    const uint32 secondBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.secondBodyEntity);

    VelocityConstraint* velocityConstraint = mVelocityConstraints + i;
    float inverseMassA = velocityConstraint->inverseMassA;
    float inverseInertiaA = velocityConstraint->inverseInertiaA;
    float inverseMassB = velocityConstraint->inverseMassB;
    float inverseInertiaB = velocityConstraint->inverseInertiaB;
    uint32 numPoints = velocityConstraint->numPoints;

    Vector2 linearVelocityA = mBodyComponents.mLinearVelocitiesConstrained[firstBodyIndex];
    float angularSpeedA = mBodyComponents.mAngularSpeedsConstrained[firstBodyIndex];
    Vector2 linearVelocityB = mBodyComponents.mLinearVelocitiesConstrained[secondBodyIndex];
    float angularSpeedB = mBodyComponents.mAngularSpeedsConstrained[secondBodyIndex];
    
    Vector2 normal = velocityConstraint->normal;
    Vector2 tangent = cross(normal, 1.0f);

    for(uint32 j = 0; j < numPoints; j++) {
      VelocityConstraint::VelocityConstraintPoint* constraintPoint = velocityConstraint->points + j;
      Vector2 P = constraintPoint->normalImpulse * normal + constraintPoint->tangentImpulse * tangent;
      angularSpeedA -= inverseInertiaA * cross(constraintPoint->rA, P);
      linearVelocityA -= inverseMassA * P;
      angularSpeedB += inverseInertiaB * cross(constraintPoint->rB, P);
      linearVelocityB += inverseMassB * P;
    }

    mBodyComponents.mLinearVelocitiesConstrained[firstBodyIndex] = linearVelocityA;
    mBodyComponents.mAngularSpeedsConstrained[firstBodyIndex] = angularSpeedA;
    mBodyComponents.mLinearVelocitiesConstrained[secondBodyIndex] = linearVelocityB;
    mBodyComponents.mAngularSpeedsConstrained[secondBodyIndex] = angularSpeedB;
  }
}

/* Initialize */
void ContactSolver::initialize(DynamicArray<LocalManifold>* manifolds, TimeStep timeStep) {
  mManifolds = manifolds;
  mTimeStep = timeStep;
  mVelocityConstraints = nullptr;
  mPositionConstraints =  nullptr;
  mNumManifolds = 0;
  const uint32 numManifolds = static_cast<uint32>(mManifolds->size());

  if(!numManifolds) {
    return;
  }

  mVelocityConstraints = static_cast<VelocityConstraint*>(mMemoryStrategy.allocate(MemoryStrategy::HandlerType::Linear, sizeof(VelocityConstraint) * numManifolds));
  mPositionConstraints = static_cast<PositionConstraint*>(mMemoryStrategy.allocate(MemoryStrategy::HandlerType::Linear, sizeof(PositionConstraint) * numManifolds));
  assert(mVelocityConstraints);
  assert(mPositionConstraints);
  const uint32 numIslands = mIslands.getNumIslands();

  for(uint32 i = 0; i < numIslands; i++) {
    if(mIslands.numManifolds[i] > 0) {
      initializeIsland(i);
    }
  }

  LOG("Contact solver found " + std::to_string(mNumManifolds) + " manifold(s)");

  initializeVelocityConstraints();
  warmStart();
}

/* Solve velocity constraints */
void ContactSolver::solveVelocityConstraints() {
  for(uint32 i = 0; i < mNumManifolds; i++) {
    LocalManifold& localManifold = (*mManifolds)[i];
    assert(localManifold.info.numPoints);

    const uint32 firstBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.firstBodyEntity);
    const uint32 secondBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.secondBodyEntity);

    VelocityConstraint* velocityConstraint =  mVelocityConstraints + i;
    float inverseMassA = velocityConstraint->inverseMassA;
    float inverseInertiaA = velocityConstraint->inverseInertiaA;
    float inverseMassB = velocityConstraint->inverseMassB;
    float inverseInertiaB = velocityConstraint->inverseInertiaB;

    Vector2 linearVelocityA = mBodyComponents.mLinearVelocitiesConstrained[firstBodyIndex];
    float angularSpeedA = mBodyComponents.mAngularSpeedsConstrained[firstBodyIndex];
    Vector2 linearVelocityB = mBodyComponents.mLinearVelocitiesConstrained[secondBodyIndex];
    float angularSpeedB = mBodyComponents.mAngularSpeedsConstrained[secondBodyIndex];

    Vector2 normal = velocityConstraint->normal;
    Vector2 tangent = cross(normal, 1.0f);
    float friction = velocityConstraint->friction;
    uint32 numPoints = velocityConstraint->numPoints;

    assert(numPoints > 0 && numPoints <= MAX_MANIFOLD_POINTS);

    /* Tangent constraints */
    for(uint32 j = 0; j < numPoints; j++) {
      VelocityConstraint::VelocityConstraintPoint* constraintPoint = velocityConstraint->points + j;
      Vector2 dv = linearVelocityB + cross(angularSpeedB, constraintPoint->rB) - linearVelocityA - cross(angularSpeedA, constraintPoint->rA);
      float vt = dot(dv, tangent);
      float lambda = constraintPoint->tangentMass * (-vt);
      float maxFriction = friction * constraintPoint->normalImpulse;
      float newImpulse = clamp(constraintPoint->tangentImpulse + lambda, -maxFriction, maxFriction);
      lambda = newImpulse - constraintPoint->tangentImpulse;
      constraintPoint->tangentImpulse = newImpulse;

      Vector2 P = lambda * tangent;
      linearVelocityA -= inverseMassA * P;
      angularSpeedA -= inverseInertiaA * cross(constraintPoint->rA, P);
      linearVelocityB += inverseMassB * P;
      angularSpeedB += inverseInertiaB * cross(constraintPoint->rB, P);
    }

    /* Normal constraints */
    if(numPoints < MAX_MANIFOLD_POINTS) {
      for(uint32 j = 0; j < numPoints; j++) {
        VelocityConstraint::VelocityConstraintPoint* constraintPoint = velocityConstraint->points + j;
        Vector2 dv = linearVelocityB + cross(angularSpeedB, constraintPoint->rB) - linearVelocityA - cross(angularSpeedA, constraintPoint->rA);
        float vn = dot(dv, normal);
        float lambda = -constraintPoint->normalMass * (vn - constraintPoint->velocityBias);
        float newImpulse = std::max(constraintPoint->normalImpulse + lambda, 0.0f);
        lambda = newImpulse - constraintPoint->normalImpulse;
        constraintPoint->normalImpulse = newImpulse;

        Vector2 P = lambda * normal;
        linearVelocityA -= inverseMassA * P;
        angularSpeedA -= inverseInertiaA * cross(constraintPoint->rA, P);
        linearVelocityB += inverseMassB * P;
        angularSpeedB += inverseInertiaB * cross(constraintPoint->rB, P);
      }
    }
    else {
      VelocityConstraint::VelocityConstraintPoint* constraintPointA = velocityConstraint->points + 0;
      VelocityConstraint::VelocityConstraintPoint* constraintPointB = velocityConstraint->points + 1;

      Vector2 a(constraintPointA->normalImpulse, constraintPointB->normalImpulse);
      assert(a.x >= 0.0f && a.y >= 0.0f);

      Vector2 dv1 = linearVelocityB + cross(angularSpeedB, constraintPointA->rB) - linearVelocityA - cross(angularSpeedA, constraintPointA->rA);
      Vector2 dv2 = linearVelocityB + cross(angularSpeedB, constraintPointB->rB) - linearVelocityA - cross(angularSpeedA, constraintPointB->rA);
      
      float vn1 = dot(dv1, normal);
      float vn2 = dot(dv2, normal);

      Vector2 b;
      b.x = vn1 - constraintPointA->velocityBias;
      b.y = vn2 - constraintPointB->velocityBias;
      b -= velocityConstraint->K * a;

      while(true) {
        Vector2 x = -velocityConstraint->normalMass * b;

        if(x.x >= 0.0f && x.y >= 0.0f) {
          Vector2 d = x - a;
          Vector2 PA = d.x * normal;
          Vector2 PB = d.y * normal;

          linearVelocityA -= inverseMassA * (PA + PB);
          angularSpeedA -= inverseInertiaA * (cross(constraintPointA->rA, PA) + cross(constraintPointB->rA, PB));
          linearVelocityB += inverseMassB * (PA + PB);
          angularSpeedB += inverseInertiaB * (cross(constraintPointA->rB, PA) + cross(constraintPointB->rB, PB));

          constraintPointA->normalImpulse = x.x;
          constraintPointB->normalImpulse = x.y;
          break;
        }

        x.x = -constraintPointA->normalMass * b.x;
        x.y = 0.0f;
        vn1 = 0.0f;
        Vector2 columnA = velocityConstraint->K.getColumn(0);
        vn2 = columnA.y * x.x + b.y;

        if(x.x >= 0.0f && vn2 >= 0.0f) {
          Vector2 d = x - a;
          Vector2 PA = d.x * normal;
          Vector2 PB = d.y * normal;

          linearVelocityA -= inverseMassA * (PA + PB);
          angularSpeedA -= inverseInertiaA * (cross(constraintPointA->rA, PA) + cross(constraintPointB->rA, PB));
          linearVelocityB += inverseMassB * (PA + PB);
          angularSpeedB += inverseInertiaB * (cross(constraintPointA->rB, PA) + cross(constraintPointB->rB, PB));

          constraintPointA->normalImpulse = x.x;
          constraintPointB->normalImpulse = x.y;
          break;
        }

        x.x = 0.0f;
        x.y = -constraintPointB->normalMass * b.y;
        Vector2 columnB = velocityConstraint->K.getColumn(1);
        vn1 = columnB.x * x.y + b.x;
        vn2 = 0.0f;

        if(x.y >= 0.0f && vn1 >= 0.0f) {
          Vector2 d = x - a;
          Vector2 PA = d.x * normal;
          Vector2 PB = d.y * normal;

          linearVelocityA -= inverseMassA * (PA + PB);
          angularSpeedA -= inverseInertiaA * (cross(constraintPointA->rA, PA) + cross(constraintPointB->rA, PB));
          linearVelocityB += inverseMassB * (PA + PB);
          angularSpeedB += inverseInertiaB * (cross(constraintPointA->rB, PA) + cross(constraintPointB->rB, PB));

          constraintPointA->normalImpulse = x.x;
          constraintPointB->normalImpulse = x.y;
          break;
        }

        x.x = 0.0f;
        x.y = 0.0f;
        vn1 = b.x;
        vn2 = b.y;

        if(vn1 >= 0.0f && vn2 >= 0.0f) {
          Vector2 d = x - a;
          Vector2 PA = d.x * normal;
          Vector2 PB = d.y * normal;

          linearVelocityA -= inverseMassA * (PA + PB);
          angularSpeedA -= inverseInertiaA * (cross(constraintPointA->rA, PA) + cross(constraintPointB->rA, PB));
          linearVelocityB += inverseMassB * (PA + PB);
          angularSpeedB += inverseInertiaB * (cross(constraintPointA->rB, PA) + cross(constraintPointB->rB, PB));

          constraintPointA->normalImpulse = x.x;
          constraintPointB->normalImpulse = x.y;
          break;
        }

        /* No solution */
        break;
      }
    }

    mBodyComponents.mLinearVelocitiesConstrained[firstBodyIndex] = linearVelocityA;
    mBodyComponents.mAngularSpeedsConstrained[firstBodyIndex] = angularSpeedA;
    mBodyComponents.mLinearVelocitiesConstrained[secondBodyIndex] = linearVelocityB;
    mBodyComponents.mAngularSpeedsConstrained[secondBodyIndex] = angularSpeedB;
  }
}

/* Solve position constraints */
void ContactSolver::solvePositionConstraints() {
  float minSeparation = 0.0f;
  uint32 islandStartManifoldIndex = 0;
  uint32 islandIndex = mIslands.getIslandIndex(islandStartManifoldIndex);
  uint32 totalIslandManifolds = mIslands.numManifolds[islandIndex];
  uint32 accountedIslandManifolds = 0;

  for(uint32 i = 0; i < mNumManifolds; i++) {
    if(accountedIslandManifolds == totalIslandManifolds) {
      islandStartManifoldIndex += totalIslandManifolds;
      islandIndex = mIslands.getIslandIndex(islandStartManifoldIndex);
      totalIslandManifolds = mIslands.numManifolds[islandIndex];
      accountedIslandManifolds = 0;
    }

    LocalManifold& localManifold = (*mManifolds)[i];
    assert(localManifold.info.numPoints);

    const uint32 firstBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.firstBodyEntity);
    const uint32 secondBodyIndex = mBodyComponents.getComponentEntityIndex(localManifold.secondBodyEntity);

    PositionConstraint* positionConstraint = mPositionConstraints + i;
    Vector2 localCenterA = positionConstraint->localCenterA;
    float inverseMassA = positionConstraint->inverseMassA;
    float inverseInertiaA = positionConstraint->inverseInertiaA;
    Vector2 localCenterB = positionConstraint->localCenterB;
    float inverseMassB = positionConstraint->inverseMassB;
    float inverseInertiaB = positionConstraint->inverseInertiaB;
    uint32 numPoints = positionConstraint->numPoints;

    Vector2 positionA = mBodyComponents.mPositionsConstrained[firstBodyIndex];
    float angleA = mBodyComponents.mOrientationsConstrained[firstBodyIndex].getAngle();
    Vector2 positionB = mBodyComponents.mPositionsConstrained[secondBodyIndex];
    float angleB = mBodyComponents.mOrientationsConstrained[secondBodyIndex].getAngle();

    for(uint32 j = 0; j < numPoints; j++) {
      Transform transformA;
      Transform transformB;
      transformA.setOrientation(Rotation(angleA));
      transformB.setOrientation(Rotation(angleB));
      transformA.setPosition(positionA - (transformA.getOrientation() * localCenterA));
      transformB.setPosition(positionB - (transformB.getOrientation() * localCenterB));

      PositionSolverInfo solverInfo(positionConstraint, transformA, transformB, j);
      Vector2 normal = solverInfo.normal;
      Vector2 point = solverInfo.point;
      float separation = solverInfo.separation;
      Vector2 rA = point - positionA;
      Vector2 rB = point - positionB;
      minSeparation = std::min(minSeparation, separation);

      float C = clamp(BAUMGARTE * (separation + LINEAR_SLOP), -MAX_LINEAR_CORRECTION, 0.0f);
      float rnA = cross(rA, normal);
      float rnB = cross(rB, normal);
      float K = inverseMassA + inverseMassB + inverseInertiaA * rnA * rnA + inverseInertiaB * rnB * rnB;
      float impulse = K > 0.0f ? -C / K: 0.0f;

      Vector2 P = impulse * normal;
      positionA -= inverseMassA * P;
      angleA -= inverseInertiaA * cross(rA, P);
      positionB += inverseMassB * P;
      angleB += inverseInertiaB * cross(rB, P);
    }

    mBodyComponents.mPositionsConstrained[firstBodyIndex] = positionA;
    mBodyComponents.mOrientationsConstrained[firstBodyIndex] = Rotation(angleA);
    mBodyComponents.mPositionsConstrained[secondBodyIndex] = positionB;
    mBodyComponents.mOrientationsConstrained[secondBodyIndex] = Rotation(angleB);

    mIslands.solved[islandIndex] = mIslands.solved[islandIndex] && (minSeparation >= -3.0f * LINEAR_SLOP);
    accountedIslandManifolds++;
  }
}

/* Store impulses for warm starting in the next frame */
void ContactSolver::storeImpulses() {
  for(uint32 i = 0; i < mNumManifolds; i++) {
    VelocityConstraint* velocityConstraint = mVelocityConstraints + i;

    for(uint32 j = 0; j < velocityConstraint->numPoints; j++) {
      (*mManifolds)[i].info.points[j].normalImpulse = velocityConstraint->points[j].normalImpulse;
      (*mManifolds)[i].info.points[j].tangentImpulse = velocityConstraint->points[j].tangentImpulse;
    }
  }
}

/* Release allocated memory */
void ContactSolver::reset() {
  if(mManifolds->size()) {
    mMemoryStrategy.free(MemoryStrategy::HandlerType::Linear, mVelocityConstraints, sizeof(VelocityConstraint) * mManifolds->size());
    mMemoryStrategy.free(MemoryStrategy::HandlerType::Linear, mPositionConstraints, sizeof(PositionConstraint) * mManifolds->size());
  }
}
