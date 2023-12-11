#ifndef PHYSICS_COLLISION_ALGORITHM_H
#define PHYSICS_COLLISION_ALGORITHM_H

#include <physics/Configuration.h>
#include <physics/collision/Contact.h>
#include <physics/collision/NarrowPhase.h>

namespace physics {

/* Forward declarations */
class NarrowPhase;
struct LocalManifoldInfo;

class CollisionAlgorithm {

  public:
    /* -- Methods -- */

    /* Constructor */
    CollisionAlgorithm() = default;

    /* Destructor */
    virtual ~CollisionAlgorithm() = default;

    /* Deleted copy constructor */
    CollisionAlgorithm(const CollisionAlgorithm& algorithm) = delete;

    /* Deleted assignment operator */
    CollisionAlgorithm& operator=(const CollisionAlgorithm& algorithm) = delete;

    /* Execute the collision algorithm */
    virtual void execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold)=0;
};

}

#endif