#ifndef PHYSICS_DYNAMICS_H
#define PHYSICS_DYNAMICS_H

#include <physics/common/BodyComponents.h>
#include <physics/common/ColliderComponents.h>
#include <physics/common/TransformComponents.h>

namespace physics {

/* Forward declarations */
class World;
struct TimeStep;

class Dynamics {

  private:
    /* -- Attributes -- */

    /* World */
    World& mWorld;

    /* Body components */
    BodyComponents& mBodyComponents;

    /* Collider components */
    ColliderComponents& mColliderComponents;

    /* Transform components */
    TransformComponents& mTransformComponents;

    /* Gravity enabled/disabled */
    bool& mIsGravityEnabled;

    /* Gravity */
    Vector2& mGravity;

  public:
    /* -- Methods -- */
    
    /* Constructor */
    Dynamics(World& world,
             BodyComponents& bodyComponents,
             ColliderComponents& colliderComponents,
             TransformComponents& transformComponents,
             bool& isGravityEnabled,
             Vector2& gravity);

    /* Destructor */
    ~Dynamics() = default;

    /* Initialize constrained positions and orientations */
    void initializeStateConstraints();

    /* Integrate velocities of bodies */
    void integrateVelocities(TimeStep timeStep);

    /* Integrate positions of bodies */
    void integratePositions(TimeStep timeStep);

    /* Update the states of the bodies */
    void updateBodyStates();

    /* Clear the forces and torques acting on each body */
    void resetExternalStimuli();
};

}

#endif