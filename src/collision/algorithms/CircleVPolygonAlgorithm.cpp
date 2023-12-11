#include <physics/collision/algorithms/CircleVPolygonAlgorithm.h>

using namespace physics;

/* Execute the collision algorithm */
void CircleVPolygonAlgorithm::execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) {
  /* Extract prerequisite information from the narrow phase input */
  assert(!narrowPhase.entries[entryIndex].isColliding);
  const Transform& firstTransform = narrowPhase.entries[entryIndex].firstShapeTransform;
  const Transform& secondTransform = narrowPhase.entries[entryIndex].secondShapeTransform;
  const PolygonShape* firstShape = dynamic_cast<PolygonShape*>(narrowPhase.entries[entryIndex].firstShape);
  const CircleShape* secondShape = dynamic_cast<CircleShape*>(narrowPhase.entries[entryIndex].secondShape);
  manifold.numPoints = 0;

  /* Transform the circle's position to the polygon's frame of reference */
  Vector2 c = secondTransform * secondShape->getCentroid();
  Vector2 cLocal = firstTransform ^ c;

  uint32 normalIndex = 0;
  float separation = -FLOAT_LARGEST;
  float radius = firstShape->getRadius() + secondShape->getRadius();
  uint32 numVertices = firstShape->getNumVertices();
  const Vector2* vertices = firstShape->mVertices;
  const Vector2* normals = firstShape->mNormals;

  /* Find the minimum separating edge */
  for(uint32 i = 0; i < numVertices; i++) {
    float s = dot(normals[i], cLocal - vertices[i]);

    /* No collision */
    if(s > radius) {
      return;
    }

    if(s > separation) {
      separation = s;
      normalIndex = i;
    }
  }

  /* Vertices subtending incident face */
  uint32 firstVertexIndex = normalIndex;
  uint32 secondVertexIndex = firstVertexIndex + 1 < numVertices ? firstVertexIndex + 1 : 0;
  Vector2 firstVertex = vertices[firstVertexIndex];
  Vector2 secondVertex = vertices[secondVertexIndex];

  /* Center is inside polygon */
  if(separation < FLOAT_EPSILON) {
    /* Populate the local manifold with the relevant collision info for the contact solver */
    manifold.numPoints = 1;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceA;
    manifold.localNormal = normals[normalIndex];
    manifold.localPoint = 0.5f * (firstVertex + secondVertex);
    manifold.points[0].localPoint = secondShape->getCentroid();
    manifold.points[0].info.key = 0;
    narrowPhase.entries[entryIndex].isColliding = true;
    return;
  }

  /* Barycentric coordinates */
  float u1 = dot(cLocal - firstVertex, secondVertex - firstVertex);
  float u2 = dot(cLocal - secondVertex, firstVertex - secondVertex);

  if(u1 <= 0.0f) {
    if(cLocal.distanceSquare(firstVertex) > square(radius)) {
      return;
    }

    /* Populate the local manifold with the relevant collision info for the contact solver */
    manifold.numPoints = 1;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceA;
    manifold.localNormal = cLocal - firstVertex;
    manifold.localNormal.normalize();
    manifold.localPoint = firstVertex;
    manifold.points[0].localPoint = secondShape->getCentroid();
    manifold.points[0].info.key = 0;
    narrowPhase.entries[entryIndex].isColliding = true;
  }
  else if(u2 <= 0.0f) {
    if(cLocal.distanceSquare(secondVertex) > square(radius)) {
      return;
    }

    /* Populate the local manifold with the relevant collision info for the contact solver */
    manifold.numPoints = 1;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceA;
    manifold.localNormal = cLocal - secondVertex;
    manifold.localNormal.normalize();
    manifold.localPoint = secondVertex;
    manifold.points[0].localPoint = secondShape->getCentroid();
    manifold.points[0].info.key = 0;
    narrowPhase.entries[entryIndex].isColliding = true;
  }
  else {
    Vector2 faceCenter = 0.5f * (firstVertex + secondVertex);
    float s = dot(cLocal - faceCenter, normals[firstVertexIndex]);

    if(s > radius) {
      return;
    }

    /* Populate the local manifold with the relevant collision info for the contact solver */
    manifold.numPoints = 1;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceA;
    manifold.localNormal = normals[firstVertexIndex];
    manifold.localPoint = faceCenter;
    manifold.points[0].localPoint = secondShape->getCentroid();
    manifold.points[0].info.key = 0;
    narrowPhase.entries[entryIndex].isColliding = true;
  }
}