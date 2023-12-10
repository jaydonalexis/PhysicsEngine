#ifndef PHYSICS_SHAPE_H
#define PHYSICS_SHAPE_H

#include <physics/Configuration.h>
#include <physics/collections/DynamicArray.h>
#include <physics/mathematics/Math.h>
#include <cassert>

#define SHAPE_TYPES 3

namespace physics {

/* Types of shapes */
enum class ShapeType {Circle, Polygon, Edge};

/* Forward declarations */
class AABB;
class Collider;
class Body;
struct Vector2;

class Shape {

  protected:
    /* -- Attributes -- */

    /* Type of the shape */
    ShapeType mType;

    /* Colliders associated with this specific shape */
    DynamicArray<Collider*> mColliders;

    /* Shape radius */
    float mRadius;

    /* -- Methods -- */

    /* Get the size of the shape in bytes */
    virtual size_t byteSize() const=0;
    
    /* Query whether a point is inside the shape */
    virtual bool testPoint(const Vector2& pointLocal) const=0;

    /* Add a new collider to the particular shape */
    void addCollider(Collider* collider);

    /* Remove a collider from the particular shape */
    void removeCollider(Collider* collider);

    /* Alert colliders that the shape size has changed */
    void alertSizeChange();

  public:
    /* -- Methods -- */

    /* Constructor */
    Shape(const ShapeType& type, float radius, MemoryHandler& memoryHandler);

    /* Destructor */
    virtual ~Shape() = default;

    /* Deleted copy constructor */
    Shape(const Shape& shape) = delete;

    /* Deleted assignment operator */
    Shape& operator=(const Shape& shape) = delete;

    /* Get the type of the shape */
    const ShapeType& getType() const;

    /* Get the radius of the shape */
    float getRadius() const;

    /* Get the rotational inertia of the shape about the local origin */
    virtual float getLocalInertia(float mass) const=0;

    /* Get the area of the shape */
    virtual float getArea() const=0;

    /* Get the centroid of the shape */
    virtual Vector2 getCentroid() const=0;

    /* Get the local bounds of the shape */
    virtual void getLocalBounds(Vector2& lowerBound, Vector2& upperBound) const=0;

    /* Compute the world space AABB of the shape */
    virtual void computeAABB(AABB& aabb, const Transform& transform) const=0;

    /* -- Friends -- */
    friend class Collider;
    friend class Body;
    friend class World;
    friend class BroadPhase;
};

}

#endif