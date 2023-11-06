#ifndef PHYSICS_CONVEX_HULL_H
#define PHYSICS_CONVEX_HULL_H

#include <physics/Configuration.h>
#include <physics/mathematics/Math.h>

namespace physics {

struct Hull {

  public:
    /* -- Attributes -- */

    /* Hull points */
    Vector2 points[MAX_POLYGON_VERTICES];

    /* Number of hull points */
    uint32 numPoints;
};

/* Debug */
Hull computeHull(const Vector2* points, uint32 numPoints);

/* Recursive divide and conquer method for quick hull algorithm */
Hull recurseHull(const Vector2& minPoint, const Vector2& maxPoint, const Vector2* points, uint32 numPoints);
}

#endif