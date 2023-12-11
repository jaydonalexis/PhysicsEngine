#ifndef PHYSICS_ALGORITHM_DISPATCH_H
#define PHYSICS_ALGORITHM_DISPATCH_H

#include <physics/collision/Shape.h>
#include <physics/collision/algorithms/CollisionAlgorithm.h>
#include <physics/collision/algorithms/CircleVCircleAlgorithm.h>
#include <physics/collision/algorithms/CircleVPolygonAlgorithm.h>
#include <physics/collision/algorithms/PolygonVPolygonAlgorithm.h>

namespace physics {

enum class CollisionAlgorithmType {CircleVCircle, CircleVPolygon, PolygonVPolygon};

class AlgorithmDispatch {

  private:
    /* -- Attributes -- */

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* Sphere-Sphere algorithm */
    CircleVCircleAlgorithm* mCircleVCircleAlgorithm;

    /* Sphere-Polygon algorithm */
    CircleVPolygonAlgorithm* mCircleVPolygonAlgorithm;

    /* Polygon-Polygon algorithm */
    PolygonVPolygonAlgorithm* mPolygonVPolygonAlgorithm;

    /* Collision algorithm matrix */
    CollisionAlgorithmType mCollisionMatrix[NUM_SHAPE_TYPES][NUM_SHAPE_TYPES];

    /* Populate collision matrix */
    void populateCollisionMatrix();

  public:
    /* -- Methods -- */
    
    /* Constructor */
    AlgorithmDispatch(MemoryHandler& memoryHandler);
    
    /* Destructor */
    ~AlgorithmDispatch();

    /* Get the collision algorithm type */
    CollisionAlgorithmType getCollisionAlgorithmType(const ShapeType& firstShapeType, const ShapeType& secondShapeType) const;

    /* Get the collision algorithm */
    CollisionAlgorithm* getCollisionAlgorithm(const CollisionAlgorithmType& algorithmType) const;
};

}

#endif