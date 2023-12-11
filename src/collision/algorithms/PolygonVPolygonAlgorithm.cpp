#include <physics/collision/algorithms/PolygonVPolygonAlgorithm.h>

using namespace physics;

/* Get the maximum separation between the two polygons using the edge normals of the first polygon */
float PolygonVPolygonAlgorithm::getMaxSeparation(const PolygonShape* firstShape, const PolygonShape* secondShape, const Transform& firstTransform, const Transform& secondTransform, uint32* edgeIndex) {
  uint32 firstNumVertices = firstShape->getNumVertices();
  uint32 secondNumVertices = secondShape->getNumVertices();
  const Vector2* firstNormals = firstShape->mNormals;
  const Vector2* firstVertices = firstShape->mVertices;
  const Vector2* secondVertices = secondShape->mVertices;
  Transform transform = secondTransform ^ firstTransform;

  uint32 bestIndex = 0;
  float maxSeparation = -FLOAT_LARGEST;

  for(uint32 i = 0; i < firstNumVertices; i++) {
    /* Transform the normal of the first polygon to the frame of the second polygon */
    Vector2 normal = transform.getOrientation() * firstNormals[i];
    Vector2 firstVertex = transform * firstVertices[i];
    float firstSeparation = FLOAT_LARGEST;

    /* Find the deepest point for the ith normal */
    for(uint32 j = 0; i < secondNumVertices; j++) {
      float secondSeparation = dot(normal, secondVertices[j] - firstVertex);

      if(secondSeparation < firstSeparation) {
        firstSeparation = secondSeparation;
      }
    }

    if(firstSeparation > maxSeparation) {
      maxSeparation = firstSeparation;
      bestIndex = i;
    }
  }

  *edgeIndex = bestIndex;
  return maxSeparation;
}

/* Get incident edge */
void PolygonVPolygonAlgorithm::getIncidentEdge(const PolygonShape* firstShape, const PolygonShape* secondShape, const Transform& firstTransform, const Transform& secondTransform, uint32 firstEdge, ClipVertex vertices[MAX_MANIFOLD_POINTS]) {
  const Vector2* firstNormals = firstShape->mNormals;
  uint32 secondNumVertices = secondShape->getNumVertices();
  const Vector2* secondVertices = secondShape->mVertices;
  const Vector2* secondNormals = secondShape->mNormals;
  assert(0 <= firstEdge && firstEdge < firstShape->getNumVertices());

  /* Transform the normal of the reference edge to the frame of the second polygon */
  Vector2 firstNormal = secondTransform ^ (firstTransform.getOrientation() * firstNormals[firstEdge]);

  /* Find the incident edge on the second polygon */
  uint32 index = 0;
  float minDot = FLOAT_LARGEST;

  for(uint32 i = 0; i < secondNumVertices; i++) {
    float firstDot = dot(firstNormal, secondNormals[i]);

    if(firstDot < minDot) {
      minDot = firstDot;
      index = i;
    }
  }

  uint32 firstIndex = index;
  uint32 secondIndex = firstIndex + 1 < secondNumVertices ? firstIndex + 1 : 0;

  /* Construct the clip vertices for the incident edge */
  vertices[0].vertex = secondTransform * secondVertices[firstIndex];
  vertices[0].info.feature.firstIndex = static_cast<uint8>(firstEdge);
  vertices[0].info.feature.secondIndex = static_cast<uint8>(firstIndex);
  vertices[0].info.feature.firstType = ContactFeature::FeatureType::Face;
  vertices[0].info.feature.secondType = ContactFeature::FeatureType::Vertex;

  vertices[1].vertex = secondTransform * secondVertices[secondIndex];
  vertices[1].info.feature.firstIndex = static_cast<uint8>(firstEdge);
  vertices[1].info.feature.secondIndex = static_cast<uint8>(secondIndex);
  vertices[1].info.feature.firstType = ContactFeature::FeatureType::Face;
  vertices[1].info.feature.secondType = ContactFeature::FeatureType::Vertex;
}

/* Clip segment to line */
uint32 PolygonVPolygonAlgorithm::clipToLine(const ClipVertex verticesInput[MAX_MANIFOLD_POINTS], ClipVertex verticesOutput[MAX_MANIFOLD_POINTS], const Vector2& normal, float offset, uint32 firstVertexIndex) {
  uint32 numPoints = 0;

  /* Distance of end points to the line */
  float firstDistance = dot(normal, verticesInput[0].vertex) - offset;
  float secondDistance = dot(normal, verticesInput[1].vertex) - offset;

  /* First point is behind the plane */
  if(firstDistance <= 0.0f) {
    verticesOutput[numPoints++] = verticesInput[0];
  }

  /* Second point is behind the plane */
  if(secondDistance <= 0.0f) {
    verticesOutput[numPoints++] = verticesInput[1];
  }

  /* Both points are on different sides of the plane */
  if(firstDistance * secondDistance < 0.0f) {
    /* Edge-plane intersection */
    float interpolation = firstDistance / (firstDistance - secondDistance);
    verticesOutput[numPoints].vertex = verticesInput[0].vertex + interpolation * (verticesInput[1].vertex - verticesInput[0].vertex);

    /* First vertex is in contact with the second edge */
    verticesOutput[numPoints].info.feature.firstIndex = static_cast<uint8>(firstVertexIndex);
    verticesOutput[numPoints].info.feature.secondIndex = verticesInput[0].info.feature.secondIndex;
    verticesOutput[numPoints].info.feature.firstType = ContactFeature::FeatureType::Vertex;
    verticesOutput[numPoints].info.feature.secondType = ContactFeature::FeatureType::Face;
    numPoints++;
    assert(numPoints == 2);
  }

  return numPoints;
}

/* Execute the collision algorithm */
void PolygonVPolygonAlgorithm::execute(NarrowPhase& narrowPhase, uint32 entryIndex, LocalManifoldInfo& manifold) {
  /* Extract prerequisite information from the narrow phase input */
  assert(!narrowPhase.entries[entryIndex].isColliding);
  const Transform& firstTransform = narrowPhase.entries[entryIndex].firstShapeTransform;
  const Transform& secondTransform = narrowPhase.entries[entryIndex].secondShapeTransform;
  const PolygonShape* firstShape = dynamic_cast<PolygonShape*>(narrowPhase.entries[entryIndex].firstShape);
  const PolygonShape* secondShape = dynamic_cast<PolygonShape*>(narrowPhase.entries[entryIndex].secondShape);
  manifold.numPoints = 0;

  float radius = firstShape->getRadius() + secondShape->getRadius();
  uint32 firstEdge = 0;
  float firstSeparation = getMaxSeparation(firstShape, secondShape, firstTransform, secondTransform, &firstEdge);

  if(firstSeparation > radius) {
    return;
  }

  uint32 secondEdge = 0;
  float secondSeparation = getMaxSeparation(secondShape, firstShape, secondTransform, firstTransform, &secondEdge);

  if(secondSeparation > radius) {
    return;
  }

  const PolygonShape* referencePolygon;
  const PolygonShape* incidentPolygon;
  Transform xf1;
  Transform xf2;
  uint32 referenceEdge;
  bool flip;
  const float kTol = 0.1f * LINEAR_SLOP;

  if(secondSeparation > firstSeparation + kTol) {
    referencePolygon = secondShape;
    incidentPolygon = firstShape;
    xf1 = secondTransform;
    xf2 = firstTransform;
    referenceEdge = secondEdge;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceB;
    flip = true;
  }
  else {
    referencePolygon = firstShape;
    incidentPolygon = secondShape;
    xf1 = firstTransform;
    xf2 = secondTransform;
    referenceEdge = firstEdge;
    manifold.type = LocalManifoldInfo::ManifoldType::FaceA;
    flip = false;
  }

  ClipVertex incidentEdge[MAX_MANIFOLD_POINTS];
  getIncidentEdge(referencePolygon, incidentPolygon, xf1, xf2, referenceEdge, incidentEdge);

  uint32 firstNumVertices = referencePolygon->getNumVertices();
  const Vector2* firstVertices = referencePolygon->mVertices;

  uint32 iv1 = referenceEdge;
  uint32 iv2 = referenceEdge + 1 < firstNumVertices ? referenceEdge + 1 : 0;

  Vector2 v11 = firstVertices[iv1];
  Vector2 v12 = firstVertices[iv2];
  Vector2 localTangent = v12 - v11;
  localTangent.normalize();

  Vector2 localNormal = cross(localTangent, 1.0f);
  Vector2 planePoint = 0.5f * (v11 + v12);

  Vector2 tangent = xf1.getOrientation() * localTangent;
  Vector2 normal = cross(tangent, 1.0f);

  v11 = xf1 * v11;
  v12 = xf1 * v12;

  /* Face offset and side offsets extended by skin thickness */
  float frontOffset = dot(normal, v11);
  float firstSideOffset = -dot(tangent, v11) + radius;
  float secondSideOffset = dot(tangent, v12) + radius;

  /* Clip incident edge against extruded reference edge side edges */
  ClipVertex firstClipPoints[MAX_MANIFOLD_POINTS];
  ClipVertex secondClipPoints[MAX_MANIFOLD_POINTS];
  uint32 np;

  np = clipToLine(incidentEdge, firstClipPoints, -tangent, firstSideOffset, iv1);

  if(np < MAX_MANIFOLD_POINTS) {
    return;
  }

  np = clipToLine(firstClipPoints, secondClipPoints, tangent, secondSideOffset, iv2);

  if(np < MAX_MANIFOLD_POINTS) {
    return;
  }

  manifold.localNormal = localNormal;
  manifold.localPoint = planePoint;
  uint32 numPoints = 0;

  for(uint32 i = 0; i < MAX_MANIFOLD_POINTS; i++) {
    float separation = dot(normal, secondClipPoints[i].vertex) - frontOffset;

    if(separation <= radius) {
      ContactPoint* contactPoint = manifold.points + numPoints;
      contactPoint->localPoint = xf2 ^ secondClipPoints[i].vertex;
      contactPoint->info = secondClipPoints[i].info;

      if(flip) {
        ContactFeature contactFeature = contactPoint->info.feature;
        contactPoint->info.feature.firstIndex = contactFeature.secondIndex;
        contactPoint->info.feature.secondIndex = contactFeature.firstIndex;
        contactPoint->info.feature.firstType = contactFeature.secondType;
        contactPoint->info.feature.secondType = contactFeature.firstType;
      }

      numPoints++;
    }
  }

  manifold.numPoints = numPoints;
  /* Debug */
  narrowPhase.entries[entryIndex].isColliding = numPoints > 0;
}