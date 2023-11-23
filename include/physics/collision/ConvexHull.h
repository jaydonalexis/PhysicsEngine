#ifndef PHYSICS_CONVEX_HULL_H
#define PHYSICS_CONVEX_HULL_H

#include <physics/Configuration.h>
#include <physics/mathematics/Math.h>

namespace physics {

class Hull {

  private:
    /* -- Attributes -- */

    /* Hull points */
    Vector2 mPoints[MAX_POLYGON_VERTICES];

    /* Number of hull points */
    uint32 mNumPoints;

  public:
    /* -- Methods -- */

    /* Constructor */
    Hull() = default;

    /* Constructor */
    Hull(const Vector2* points, uint32 numPoints);

    /* Initialize the convex hull */
    void init(const Vector2* points, uint32 numpoints);

    /* Recursive divide and conquer quick hull algorithm implementation */
    Hull recurse(const Vector2& minPoint, const Vector2& maxPoint, const Vector2* points, uint32 numPoints);

    /* -- Friends -- */
    friend class PolygonShape;
};

}

#endif