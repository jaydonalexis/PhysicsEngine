#ifndef PHYSICS_EDGE_SHAPE_H
#define PHYSICS_EDGE_SHAPE_H

#include <physics/collision/Shape.h>

namespace physics {

class EdgeShape : public Shape {

  protected:
    /* -- Attributes -- */

    /* Vertices for the edge */
    Pair<Vector2, Vector2> mVertices;

    /* -- Methods -- */

    /* Constructor */
    EdgeShape(const Vector2& vertex0, const Vector2& vertex1, MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~EdgeShape() override = default;

    /* Get the size of the shape in bytes */
    virtual size_t byteSize() const override;

    /* Query whether a point is inside the shape */
    virtual bool testPoint(const Vector2& pointLocal) const override;

  public:
    /* -- Methods -- */
    
    /* Deleted copy constructor */
    EdgeShape(const EdgeShape& shape) = delete;

    /* Deleted assignment operator */
    EdgeShape& operator=(const EdgeShape& shape) = delete;

    /* Set the geometric properties of the edge */
    void set(const Vector2& vertex0, const Vector2& vertex1);

    /* Debug */
    /* const reference? */
    /* Get the vertices of the edge */
    Pair<Vector2, Vector2> getVertices() const;

    /* Get the rotational inertia of the shape about the local origin */
    virtual float getLocalInertia(float mass) const override;

    /* Get the area of the shape */
    virtual float getArea() const override;

    /* Get the centroid of the shape */
    virtual Vector2 getCentroid() const override;

    /* Get the local bounds of the shape */
    virtual void getLocalBounds(Vector2& lowerBound, Vector2& upperBound) const override;

    /* Compute the world space AABB of the shape */
    virtual void computeAABB(AABB& aabb, const Transform& transform) const override;

    /* -- Friends -- */

    friend class Factory;
};

}

#endif