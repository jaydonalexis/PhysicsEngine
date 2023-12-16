#ifndef PHYSICS_CONTACT_SOLVER_H
#define PHYSICS_CONTACT_SOLVER_H

#include <physics/Configuration.h>
#include <physics/mathematics/Vector2.h>
#include <physics/mathematics/Matrix22.h>
#include <physics/collections/DynamicArray.h>
#include <physics/dynamics/Material.h>
#include <physics/collision/Contact.h>
#include <physics/common/TimeStep.h>

namespace physics {

/* Forward declarations */
class Islands;
class BodyComponents;
class ColliderComponents;
class TransformComponents;

class ContactSolver {

  private:
    /* -- Nested Classes -- */

    /* Velocity constraint */
    struct VelocityConstraint {
      
      public:
        /* -- Nested Classes -- */

        /* Velocity constraint point */
        struct VelocityConstraintPoint {

          public:
            /* -- Attributes -- */

            /* Debug */
            /* Vector from first body center to contact point */
            Vector2 rA;

            /* Debug */
            /* Vector from second body center to contact point */
            Vector2 rB;

            /* Normal mass */
            float normalMass;

            /* Tangent mass */
            float tangentMass;

            /* Normal impulse */
            float normalImpulse;

            /* Tangent impulse */
            float tangentImpulse;

            /* Velocity bias */
            float velocityBias;
        };

      /* -- Attributes -- */

      /* Velocity constraint points */
      VelocityConstraintPoint points[MAX_MANIFOLD_POINTS];

      /* Normal */
      Vector2 normal;

      /* Normal mass */
      Matrix22 normalMass;

      /* K matrix */
      Matrix22 K;

      /* First index */
      uint32 indexA;

      /* Second index */
      uint32 indexB;

      /* First inverse mass */
      float inverseMassA;

      /* Second inverse mass */
      float inverseMassB;

      /* First inverse inertia */
      float inverseInertiaA;

      /* Second inverse inertia */
      float inverseInertiaB;

      /* Friction */
      float friction;

      /* Restitution */
      float restitution;

      /* Number of points */
      uint32 numPoints;
    };

    /* Position constraint */
    struct PositionConstraint {

      public:
        /* -- Attributes -- */
      
        /* Points */
        Vector2 points[MAX_MANIFOLD_POINTS];

        /* Local normal */
        Vector2 localNormal;

        /* Local point */
        Vector2 localPoint;

        /* First index */
        uint32 indexA;

        /* Second index */
        uint32 indexB;

        /* First inverse mass */
        float inverseMassA;

        /* Second inverse mass */
        float inverseMassB;

        /* First inverse inertia */
        float inverseInertiaA;

        /* Second inverse inertia */
        float inverseInertiaB;

        /* First local center */
        Vector2 localCenterA;

        /* Second local center */
        Vector2 localCenterB;

        /* First radius */
        float radiusA;

        /* Second radius */
        float radiusB;

        /* Manifold type */
        LocalManifoldInfo::ManifoldType type;

        /* Number of points */
        uint32 numPoints;
    };

    struct PositionSolverInfo {
      
      public:
        /* -- Attributes -- */

        /* Normal */
        Vector2 normal;

        /* Point */
        Vector2 point;

        /* Separation */
        float separation;

        /* -- Methods -- */

        /* Constructor */
        PositionSolverInfo(PositionConstraint* positionConstraint, const Transform& transformA, const Transform& transformB, uint32 index) {
          assert(positionConstraint->numPoints > 0);

          switch(positionConstraint->type) {
            case LocalManifoldInfo::ManifoldType::Circles:
            {
              Vector2 pointA = transformA * positionConstraint->localPoint;
              Vector2 pointB = transformB * positionConstraint->points[0];
              normal = pointB - pointA;
              normal.normalize();
              point = 0.5f * (pointA + pointB);
              separation = dot(pointB - pointA, normal) - positionConstraint->radiusA - positionConstraint->radiusB;
            }
            
            break;

            case LocalManifoldInfo::ManifoldType::FaceA:
            {
              normal = transformA.getOrientation() * positionConstraint->localNormal;
              Vector2 planePoint = transformA * positionConstraint->localPoint;
              Vector2 clipPoint = transformB * positionConstraint->points[index];
              separation = dot(clipPoint - planePoint, normal) - positionConstraint->radiusA - positionConstraint->radiusB;
              point = clipPoint;
            }

            break;

            case LocalManifoldInfo::ManifoldType::FaceB:
            {
              normal = transformB.getOrientation() * positionConstraint->localNormal;
              Vector2 planePoint = transformB * positionConstraint->localPoint;
              Vector2 clipPoint = transformA * positionConstraint->points[index];
              separation = dot(clipPoint - planePoint, normal) - positionConstraint->radiusA - positionConstraint->radiusB;
              point = clipPoint;
              normal = -normal;
            }

            break;
          }
        }
    };

    /* -- Constants -- */

    /* -- Attributes -- */

    /* World */
    World& mWorld;

    /* Memory strategy */
    MemoryStrategy& mMemoryStrategy;

    /* Time step */
    TimeStep mTimeStep;

    /* Islands */
    Islands& mIslands;

    /* Contact Manifolds */
    DynamicArray<LocalManifold>* mManifolds;

    /* Number of contact manifolds */
    uint32 mNumManifolds;

    /* Body components */
    BodyComponents& mBodyComponents;

    /* Collider components */
    ColliderComponents& mColliderComponents;

    /* Transform components */
    TransformComponents& mTransformComponents;

    /* Velocity constraints */
    VelocityConstraint* mVelocityConstraints;

    /* Position constraints */
    PositionConstraint* mPositionConstraints;

    /* Restitution threshold */
    float& mRestitutionThreshold;

    /* -- Methods -- */

    /* Compute the collision restituion factor */
    float computeMixedRestitution(const Material& firstMaterial, const Material& secondMaterial) const;

    /* Compute the mixed friction coefficient */
    float computeMixedFriction(const Material& firstMaterial, const Material& secondMaterial) const;

    /* Initialize contact solver for a given island */
    void initializeIsland(uint32 islandIndex);
    
    /* Initialize velocity constraints */
    void initializeVelocityConstraints();

    /* Warm start the solver */
    void warmStart();

  public:

    /* -- Methods -- */

    /* Constructor */
    ContactSolver(World& world,
                  MemoryStrategy& memoryStrategy,
                  Islands& islands,
                  BodyComponents& bodyComponents,
                  ColliderComponents& colliderComponents,
                  TransformComponents& transformComponents,
                  float& restitutionThreshold);
                  
    /* Destructor */
    ~ContactSolver() = default;

    /* Initialize */
    void initialize(DynamicArray<LocalManifold>* manifolds, TimeStep timeStep);

    /* Solve velocity constraints */
    void solveVelocityConstraints();

    /* Solve position constraints */
    void solvePositionConstraints();

    /* Store impulses for warm starting in the next frame */
    void storeImpulses();

    /* Release allocated memory */
    void reset();
};

}

#endif