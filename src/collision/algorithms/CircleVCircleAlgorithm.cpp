#include <physics/collision/algorithms/CircleVCircleAlgorithm.h>

using namespace physics;

/* Execute the collision algorithm */
void CircleVCircleAlgorithm::execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) {
  /* Extract prerequisite information from the narrow phase input */
  assert(!narrowPhase.entries[entryIndex].isColliding);
  const Transform& firstTransform = narrowPhase.entries[entryIndex].firstShapeTransform;
  const Transform& secondTransform = narrowPhase.entries[entryIndex].secondShapeTransform;
  const Shape* firstShape = narrowPhase.entries[entryIndex].firstShape;
  const Shape* secondShape = narrowPhase.entries[entryIndex].secondShape;
  manifold.numPoints = 0;

  Vector2 pA = firstTransform * firstShape->getCentroid();
  Vector2 pB = secondTransform * secondShape->getCentroid();
  Vector2 displacement = pB - pA;

  float rA = firstShape->getRadius();
  float rB = secondShape->getRadius();
  float radius = rA + rB;

  /* No collision */
  if(dot(displacement, displacement) > square(radius)) {
    return;
  }

  /* Populate the local manifold with the relevant collision info for the contact solver */
  manifold.type = LocalManifoldInfo::ManifoldType::Circles;
  manifold.localPoint = firstShape->getCentroid();
  manifold.localNormal = Vector2::getZeroVector();
  manifold.numPoints = 1;
  manifold.points[0].localPoint = secondShape->getCentroid();
  manifold.points[0].info.key = 0;
  narrowPhase.entries[entryIndex].isColliding = true;
}