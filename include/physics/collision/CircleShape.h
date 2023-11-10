#ifndef PHYSICS_CIRCLE_SHAPE_H
#define PHYSICS_CIRCLE_SHAPE_H

#include <physics/collision/Shape.h>

namespace physics {

class CircleShape : public Shape {

  protected:
    /* -- Methods -- */

    /* Constructor */
    CircleShape(float radius, MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~CircleShape() override = default; 

    /* Get the size of the shape in bytes */
    virtual size_t byteSize() const override;

    /* Query whether a point is inside the shape */
    virtual bool testPoint(const Vector2& localPoint) const override;

  public:
    /* -- Methods -- */

    /* Deleted copy constructor */
    CircleShape(const CircleShape& shape) = delete;

    /* Deleted assignment operator */
    CircleShape& operator=(const CircleShape& shape) = delete;

    /* Get radius of the sphere */
    float getRadius() const;

    /* Set radius of the sphere */
    void setRadius(float radius);

    /* Get the rotational inertia of the shape about the local origin */
    virtual float getLocalInertia(float mass) const override;

    /* Get the area of the shape */
    virtual float getArea() const override;

    /* Get the centroid of the shape */
    virtual Vector2 getCentroid() const override;

    /* Compute the world space AABB of the shape */
    virtual void computeAABB(AABB& aabb, const Transform& worldTransform) const override;

    /* -- Friends -- */

    friend class Factory;
};

}

#endif;