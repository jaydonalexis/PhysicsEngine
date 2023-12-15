#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include <physics/Configuration.h>
#include <physics/common/Logger.h>
#include <physics/mathematics/Math.h>
#include <physics/common/TimeStep.h>
#include <physics/collections/DynamicArray.h>
#include <physics/memory/MemoryStrategy.h>
#include <physics/common/EntityHandler.h>
#include <physics/common/BodyComponents.h>
#include <physics/common/ColliderComponents.h>
#include <physics/common/TransformComponents.h>
#include <physics/collision/CollisionDetection.h>
#include <physics/dynamics/Islands.h>
#include <physics/dynamics/ContactSolver.h>
#include <physics/dynamics/Dynamics.h>

namespace physics {

/* Forward declarations */
class Body;
class Factory;
class CollisionDetection;

class World {

  public:
    /* -- Nested Classes -- */

    /* Settings for the physics world */
    struct Settings {

      public:
        /* -- Attributes -- */

        /* Gravity */
        Vector2 gravity;

        /* Default restitution constant */
        float defaultRestitutionConstant;

        /* Restitution velocity threshold */
        float restitutionThreshold;

        /* Default friction constant */
        float defaultFrictionConstant;

        /* Enable/Disable sleeping bodies */
        bool isSleepingEnabled;

        /* Default linear velocity below which a body might become disabled */
        float defaultLinearVelocityForSleep;

        /* Default angular speed below which a body might become disabled */
        float defaultAngularSpeedForSleep;

        /* Minimum number of seconds that must elapse befoer a still body is considered as a sleeping body */
        float defaultSleepTime;

        /* Number of iterations to perform for velocity constraint solving */
        uint16 defaultVelocityConstraintSolverIterations;

        /* Number of iterations to perform for position constraint solving */
        uint16 defaultPositionConstraintSolverIterations;

        /* -- Methods -- */

        /* Constructor */
        Settings() {
          gravity = Vector2(0, -9.81f);
          defaultRestitutionConstant = 0.5f;
          /* Debug */
          restitutionThreshold = 1.0f;
          defaultFrictionConstant = 0.3f;
          isSleepingEnabled = true;
          defaultLinearVelocityForSleep = 0.02f;
          defaultAngularSpeedForSleep = 3.0f * (PI / 180.0f);
          defaultSleepTime = 1.0f;
          defaultVelocityConstraintSolverIterations = 10;
          defaultPositionConstraintSolverIterations = 8;
        }

        /* Destructor */
        ~Settings() = default;
    };

  protected:
    /* -- Attributes -- */

    /* Memory strategy */
    MemoryStrategy& mMemoryStrategy;

    /* World settings */
    Settings mSettings;

    /* Entity handler */
    EntityHandler mEntityHandler;

    /* Body components */
    BodyComponents mBodyComponents;

    /* Collider components */
    ColliderComponents mColliderComponents;

    /* Transform components */
    TransformComponents mTransformComponents;

    /* Collision detection */
    CollisionDetection mCollisionDetection;

    /* Bodies of the world */
    DynamicArray<Body*> mBodies;

    /* Islands of the current frame */
    Islands mIslands;

    /* Contact pairs ordered based on the islands of the current frame */
    DynamicArray<uint32> mIslandOrderedContactPairs;

    /* Contact solver */
    ContactSolver mContactSolver;

    /* Enable/Disable gravity */
    bool mIsGravityEnabled;
    
    /* Dynamics */
    Dynamics mDynamics;

    /* Velocity constraint solver iterations */
    uint16 mNumVelocitySolverIterations;

    /* Position constraint solver iterations */
    uint16 mNumPositionSolverIterations;

    /* Enable/Disable sleeping bodies */
    bool mIsSleepingEnabled;

    /* Linear velocity below which a body might become disabled */
    float mSleepLinearVelocity;

    /* Angular speed below which a body might become disabled */
    float mSleepAngularSpeed;

    /* Minimum number of seconds that must elapse befoer a still body is considered as a sleeping body */
    float mSleepTime;

    /* Previous frame inverse step */
    float mLastInverseDelta;

    /* -- Methods -- */

    /* Constructor */
    World(MemoryStrategy& memoryStrategy, Factory& factory, const Settings& settings = Settings());

    /* Destructor */
    ~World();

    /* Set a body to be disabled */
    void disableBody(Entity entity, bool isDisabled);
    
    /* Generate the islands of the current frame */
    void generateIslands();

    /* Solve the physics simulation */
    void solve(TimeStep timeStep);

    /* Set bodies to sleep as appropriate */
    void sleepBodies(TimeStep timeStep);

  public:
    /* -- Methods -- */

    /* Update the physics simulation */
    void step(float dt);

    /* Deleted copy constructor */
    World(const World& world) = delete;

    /* Deleted assignment operator */
    World& operator=(const World& world) = delete;

    /* Create a body */
    Body* createBody(const Transform& transform);

    /* Destroy a body */
    void destroyBody(Body* body);

    /* -- Friends -- */
    
    friend class Collider;
    friend class Body;
    friend class CollisionDetection;
    friend class Factory;
};

}

#endif