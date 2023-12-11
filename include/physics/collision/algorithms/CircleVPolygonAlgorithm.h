#ifndef PHYSICS_SPHERE_V_POLYGON_ALGORITHM_H
#define PHYSICS_SPHERE_V_POLYGON_ALGORITHM_H

#include <physics/collision/algorithms/CollisionAlgorithm.h>
#include <physics/collision/PolygonShape.h>
#include <physics/collision/CircleShape.h>

namespace physics {

class CircleVPolygonAlgorithm : public CollisionAlgorithm {

  public:
    /* -- Methods -- */

    /* Constructor */
    CircleVPolygonAlgorithm() = default;

    /* Destructor */
    virtual ~CircleVPolygonAlgorithm() override = default;

    /* Deleted copy constructor */
    CircleVPolygonAlgorithm(const CircleVPolygonAlgorithm& algorithm) = delete;

    /* Deleted assignment operator */
    CircleVPolygonAlgorithm& operator=(const CircleVPolygonAlgorithm& algorithm) = delete;

    /* Execute the collision algorithm */
    virtual void execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) override;
};

}

#endif