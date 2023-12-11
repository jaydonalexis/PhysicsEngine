#ifndef PHYSICS_SPHERE_V_SPHERE_ALGORITHM_H
#define PHYSICS_SPHERE_V_SPHERE_ALGORITHM_H

#include <physics/collision/algorithms/CollisionAlgorithm.h>
#include <physics/collision/CircleShape.h>

namespace physics {

class CircleVCircleAlgorithm : public CollisionAlgorithm {

  public:
    /* -- Methods -- */

    /* Constructor */
    CircleVCircleAlgorithm() = default;

    /* Destructor */
    virtual ~CircleVCircleAlgorithm() override = default;

    /* Deleted copy constructor */
    CircleVCircleAlgorithm(const CircleVCircleAlgorithm& algorithm) = delete;

    /* Deleted assignment operator */
    CircleVCircleAlgorithm& operator=(const CircleVCircleAlgorithm& algorithm) = delete;

    /* Execute the collision algorithm */
    virtual void execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) override;
};

}

#endif