#include <physics/dynamics/Dynamics.h>
#include <physics/common/TimeStep.h>

using namespace physics;

/* Constructor */
Dynamics::Dynamics(World& world,
                   BodyComponents& bodyComponents,
                   ColliderComponents& colliderComponents,
                   TransformComponents& transformComponents,
                   bool& isGravityEnabled,
                   Vector2& gravity) :
                   mWorld(world),
                   mBodyComponents(bodyComponents),
                   mColliderComponents(colliderComponents),
                   mTransformComponents(transformComponents),
                   mIsGravityEnabled(isGravityEnabled),
                   mGravity(gravity) {}

/* Initialize constrained positions and orientations  */
void Dynamics::initializeStateConstraints() {
  const uint32 numBodyComponets = mBodyComponents.getNumEnabledComponents();

  /* Initialize constrained positions and orientations for use during the contact solving process */
  for(uint32 i = 0; i < numBodyComponets; i++) {
    mBodyComponents.mPositionsConstrained[i] = mBodyComponents.mCentersOfMassWorld[i];
    mBodyComponents.mOrientationsConstrained[i] = mTransformComponents.getTransform(mBodyComponents.mBodyEntities[i]).getOrientation();
  }
}

/* Integrate velocities of bodies */
void Dynamics::integrateVelocities(TimeStep timeStep) {
  const uint32 numBodyComponents = mBodyComponents.getNumEnabledComponents();

  /* Debug */
  /* Update only dynamic bodies? */
  for(uint32 i = 0; i < numBodyComponents; i++) {
    /* Integrate velocity using force and torque */
    const Vector2& linearVelocity = mBodyComponents.mLinearVelocities[i];
    const float angularSpeed = mBodyComponents.mAngularSpeeds[i];
    /* Apply gravity only if it is enabled in the engine as well as enabled for the body itself */
    const Vector2& gravity = mIsGravityEnabled && mBodyComponents.mIsGravityEnabled[i] ? mGravity : Vector2::getZeroVector();
    /* Assign to constrained component data which will be used throughout the contact solver */
    mBodyComponents.mLinearVelocitiesConstrained[i] = linearVelocity + timeStep.delta * mBodyComponents.mInverseMasses[i] * (mBodyComponents.mForces[i] + mBodyComponents.mMasses[i] * gravity);
    mBodyComponents.mAngularSpeedsConstrained[i] = angularSpeed + timeStep.delta * mBodyComponents.mInverseInertias[i] * mBodyComponents.mTorques[i];
  }
  
  /* Apply damping computed via the differential equation dv/dt + c * v = 0 which has the solution v(t) = v0 * exp(-c * t) */
  for(uint32 i = 0; i < numBodyComponents; i++) {
    const float linearDampingFactor = mBodyComponents.mLinearDampings[i];
    const float angularDampingFactor = mBodyComponents.mAngularDampings[i];
    /* Use approximation */
    const float linearDamping = 1.0f / (1.0f + linearDampingFactor * timeStep.delta);
    const float angularDamping = 1.0f / (1.0f + angularDampingFactor * timeStep.delta);
    mBodyComponents.mLinearVelocitiesConstrained[i] *= linearDamping;
    mBodyComponents.mAngularSpeedsConstrained[i] *= angularDamping;
  }
}

/* Integrate positions of bodies */
void Dynamics::integratePositions(TimeStep timeStep) {
  const uint32 numBodyComponents = mBodyComponents.getNumEnabledComponents();

  for(uint32 i = 0; i < numBodyComponents; i++) {
    Vector2 linearVelocity = mBodyComponents.mLinearVelocitiesConstrained[i];
    float angularSpeed = mBodyComponents.mAngularSpeedsConstrained[i];
    
    Vector2 position = mBodyComponents.mPositionsConstrained[i];
    float angle = mBodyComponents.mOrientationsConstrained[i].getAngle();

    /* Sanity check large velocities */
    Vector2 translation = timeStep.delta * linearVelocity;

    if(dot(translation, translation) > MAX_TRANSLATION) {
      float ratio = MAX_TRANSLATION / translation.length();
      linearVelocity *= ratio;
    }

    /* Sanity check large angular speeds */
    float rotation = timeStep.delta * angularSpeed;

    if(rotation * rotation > MAX_ROTATION) {
      float ratio = MAX_ROTATION / std::abs(rotation);
      angularSpeed *= ratio;
    }

    /* Integrate position using constrained velocities and angular speeds */
    position += timeStep.delta * linearVelocity;
    angle += timeStep.delta * angularSpeed;

    mBodyComponents.mPositionsConstrained[i] = position;
    mBodyComponents.mOrientationsConstrained[i] = Rotation(angle);
    mBodyComponents.mLinearVelocitiesConstrained[i] = linearVelocity;
    mBodyComponents.mAngularSpeedsConstrained[i] = angularSpeed;
  }
}

/* Clear the forces and torques acting on each body */
void Dynamics::resetExternalStimuli() {
  const uint32 numBodyComponents = mBodyComponents.getNumComponents();

  for(uint32 i = 0; i < numBodyComponents; i++) {
    mBodyComponents.mForces[i].setZero();
    mBodyComponents.mTorques[i] = 0.0f;
  }
}

/* Update the states of the bodies */
void Dynamics::updateBodyStates() {
  const uint32 numBodyComponents = mBodyComponents.getNumEnabledComponents();

  for(uint32 i = 0; i < numBodyComponents; i++) {
    /* First update linear velocity and angular speed */
    mBodyComponents.mLinearVelocities[i] = mBodyComponents.mLinearVelocitiesConstrained[i];
    mBodyComponents.mAngularSpeeds[i] = mBodyComponents.mAngularSpeedsConstrained[i];

    /* Next update the center of mass position followed by the orientation */
    mBodyComponents.mCentersOfMassWorld[i] = mBodyComponents.mPositionsConstrained[i];
    const Rotation& constrainedOrientation = mBodyComponents.mOrientationsConstrained[i];
    mTransformComponents.getTransform(mBodyComponents.mBodyEntities[i]).setOrientation(constrainedOrientation);
  }

  /* Update the position component of the body's local to world transform */
  for(uint32 i = 0; i < numBodyComponents; i++) {
    Transform& transform = mTransformComponents.getTransform(mBodyComponents.mBodyEntities[i]);
    const Vector2& centerOfMassWorld = mBodyComponents.mCentersOfMassWorld[i];
    const Vector2& centerOfMassLocal = mBodyComponents.mCentersOfMassLocal[i];
    transform.setPosition(centerOfMassWorld - transform.getOrientation() * centerOfMassLocal);
  }

  const uint32 numColliderComponents = mColliderComponents.getNumEnabledComponents();

  /* Debug */
  /* Does this properly synchronize the transforms of the colliders in world space? */
  /* Synchronize the local to world transform of the current body's colliders */
  for(uint32 i = 0; i < numColliderComponents; i++) {
    mColliderComponents.mTransformsLocalWorld[i] = mTransformComponents.getTransform(mColliderComponents.mBodyEntities[i]) * mColliderComponents.mTransformsLocalBody[i];
  }
}