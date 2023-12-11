#ifndef PHYSICS_POLYGON_V_POLYGON_ALGORITHM_H
#define PHYSICS_POLYGON_V_POLYGON_ALGORITHM_H

#include <physics/collision/algorithms/CollisionAlgorithm.h>
#include <physics/collision/PolygonShape.h>

namespace physics {

class PolygonVPolygonAlgorithm : public CollisionAlgorithm {

  private:
    /* -- Nested Classes -- */

    /* Clip vertex */
  
    /* -- Methods -- */

    /* Get the maximum separation between the two polygons using the edge normals of the first polygon */
    float getMaxSeparation(const PolygonShape* firstShape, const PolygonShape* secondShape, const Transform& firstTransform, const Transform& secondTransform, uint32* edgeIndex);

    /* Get incident edge */
    void getIncidentEdge(const PolygonShape* firstShape, const PolygonShape* secondShape, const Transform& firstTransform, const Transform& secondTransform, uint32 edge, ClipVertex vertices[MAX_MANIFOLD_POINTS]);

    /* Clip segment to line */
    uint32 clipToLine(const ClipVertex verticesInput[MAX_MANIFOLD_POINTS], ClipVertex verticesOutput[MAX_MANIFOLD_POINTS], const Vector2& normal, float offset, uint32 vertexIndex);

  public:
    /* -- Methods -- */

    /* Constructor */
    PolygonVPolygonAlgorithm() = default;

    /* Destructor */
    virtual ~PolygonVPolygonAlgorithm() override = default;

    /* Deleted copy constructor */
    PolygonVPolygonAlgorithm(const PolygonVPolygonAlgorithm& algorithm) = delete;

    /* Deleted assignment operator */
    PolygonVPolygonAlgorithm& operator=(const PolygonVPolygonAlgorithm& algorithm) = delete;

    /* Execute the collision algorithm */
    virtual void execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) override;
};

}

#endif