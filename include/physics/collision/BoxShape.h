#ifndef PHYSICS_BOX_SHAPE_H
#define PHYSICS_BOX_SHAPE_H

#include <physics/collision/PolygonShape.h>

#define NUM_VERTICES_BOX 4

namespace physics {

class BoxShape : public PolygonShape {

  protected:
    /* -- Methods -- */

    /* Constructor */
    BoxShape(float hx, float hy, MemoryHandler& memoryHandler);

    /* Constructor */
    BoxShape(float hx, float hy, const Vector2& center, float angle, MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~BoxShape() override = default;

    /* Get the size of the shape in bytes */
    virtual size_t byteSize() const override;

    /* Query whether a point is inside the shape */
    virtual bool testPoint(const Vector2& pointLocal) const override;

  public:
    /* -- Methods -- */

    /* Deleted copy constructor */
    BoxShape(const BoxShape& shape) = delete;

    /* Delete assignment operator */
    BoxShape& operator=(const BoxShape& shape) = delete;

    /* Set the geometric properties of the box */
    void set(float hx, float hy);

    /* Set the geometric properties of the box */
    void set(float hx, float hy, const Vector2& center, float angle);

    /* Get the rotational inertia of the shape about the local origin */
    virtual float getLocalInertia(float mass) const override;

    /* Get the area of the shape */
    virtual float getArea() const override;

    /* Get the centroid of the shape */
    virtual Vector2 getCentroid() const override;

    /* -- Friends -- */

    friend class Factory;
};

}

#endif