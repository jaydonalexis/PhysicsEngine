#include <physics/collision/Shape.h>
#include <physics/collision/Collider.h>

using namespace physics;

/* Constructor */
Shape::Shape(const ShapeType& type, float radius, MemoryHandler& memoryHandler) : mType(type), mRadius(radius), mColliders(memoryHandler) {}

/* Add a new collider to the particular shape */
void Shape::addCollider(Collider* collider) {
  mColliders.add(collider);
}

/* Remove a collider from the particular shape */
void Shape::removeCollider(Collider* collider) {
  mColliders.remove(collider);
}

/* Alert colliders that the shape size has changed */
void Shape::alertSizeChange() {
  const uint32 numColliders = static_cast<uint32>(mColliders.size());

  for(uint32 i = 0; i < numColliders; i++) {
    mColliders[i]->setShapeSizeChanged(true);
  }
}

/* Get the type of the shape */
const ShapeType& Shape::getType() const {
  return mType;
}

/* Get the radius of the shape */
float Shape::getRadius() const {
  return mRadius;
}