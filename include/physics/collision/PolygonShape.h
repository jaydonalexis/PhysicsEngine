#ifndef PHYSICS_POLYGON_SHAPE_H
#define PHYSICS_POLYGON_SHAPE_H

#include <physics/Configuration.h>
#include <physics/collision/Shape.h>
#include <physics/collision/ConvexHull.h>

namespace physics {

/* Forward declarations */
struct Hull;

class PolygonShape : public Shape {

  private:
    /* -- Attributes -- */
    
    /* These members are declared since they are computationally expensive to derive */

    /* Centroid */
    Vector2 mCentroid;

    /* Area */
    float mArea;

    /* Inertia */
    float mInertia;

  protected:
    /* -- Attributes -- */

    /* Vertices of the convex polygon shape */
    Vector2 mVertices[MAX_POLYGON_VERTICES];

    /* Normals of the convex polygon shape */
    Vector2 mNormals[MAX_POLYGON_VERTICES];

    /* Number of vertices */
    uint32 mNumVertices;

    /* -- Methods -- */
    PolygonShape(MemoryHandler& memoryHandler);

    /* Constructor */
    PolygonShape(const Hull& hull, MemoryHandler& memoryHandler);

    /* Constructor */
    PolygonShape(const Vector2* points, uint32 numPoints, MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~PolygonShape() override = default;

    /* Get the size of the shape in bytes */
    virtual size_t byteSize() const override;

    /* Query whether a point is inside the shape */
    virtual bool testPoint(const Vector2& localPoint) const override;

    /* Compute geometric properties of the shape */
    void computeGeometricProperties();

  public:
    /* -- Methods -- */

    /* Deleted copy constructor */
    PolygonShape(const PolygonShape& shape) = delete;

    /* Deleted assignment operator */
    PolygonShape& operator=(const PolygonShape& shape) = delete;

    /* Set the geometric properties of the polygon */
    void set(const Hull& hull);

    /* Set the geometric properties of the polygon */
    void set(const Vector2* points, uint32 numPoints);

    /* Get the number of vertices of the polygon */
    virtual uint32 getNumVertices() const;

    /* Get the position of a given vertex */
    virtual Vector2 getVertexPosition(uint32 vertexIndex) const;

    /* Get the normal vector of a given edge */
    virtual Vector2 getEdgeNormal(uint32 edgeIndex);

    /* Get the rotational inertia of the shape about the local origin */
    virtual float getLocalInertia(float mass) const override;

    /* Get the area of the shape */
    virtual float getArea() const override;

    /* Get the centroid of the shape */
    virtual Vector2 getCentroid() const override;

    /* Compute the world space AABB of the shape */
    virtual void computeAABB(AABB& aabb, const Transform& transform) const override;

    /* -- Friends -- */

    friend class Factory;    
};

}

#endif