#include <physics/collision/ConvexHull.h>
#include <physics/collision/AABB.h>
#include <cassert>

using namespace physics;

Hull computeHull(const Vector2* points, uint32 numPoints) {
  Hull hull;
  hull.numPoints = 0;

  if(numPoints < MIN_POLYGON_VERTICES || numPoints > MAX_POLYGON_VERTICES) {
    return hull;
  }

  numPoints = std::min(numPoints, (uint32)MAX_POLYGON_VERTICES);
  AABB aabb(Vector2(FLT_MAX, FLT_MAX), Vector2(-FLT_MAX, -FLT_MAX));
  Vector2 candidates[MAX_POLYGON_VERTICES];
  uint32 numValid = 0;

  for(uint32 i = 0; i < numPoints; i++) {
    aabb.setLowerBound(min(aabb.getlowerBound(), points[i]));
    aabb.setUpperBound(max(aabb.getUpperBound(), points[i]));
    bool valid = true;
    Vector2 point1 = points[i];

    for(uint32 j = 0; j < i; j++) {
      Vector2 point2 = points[j];
      
      if(point1.distanceSquare(point2) < QUICK_HULL_WELD_TOLERANCE) {
        valid = false;
        break;
      }
    }

    if(valid) {
      candidates[numValid++] = point1;
    }
  }

  if(numValid < MIN_POLYGON_VERTICES) {
    return hull;
  }

  /* First extreme point for quick hull algorithm */
  Vector2 center = aabb.getCenter();
  uint32 i1 = 0;
  float ds1 = center.distanceSquare(candidates[i1]);

  for(uint32 i = 1; i < numValid; i++) {
    float ds2 = center.distanceSquare(candidates[i]);

    if(ds2 > ds1) {
      i1 = i;
      ds1 = ds2;
    }
  }

  Vector2 point1 = candidates[i1];
  candidates[i1] = candidates[numValid--];

  /* Second extreme point for quick hull algorithm */
  uint32 i2 = 0;
  ds1 = point1.distanceSquare(candidates[i2]);

  for(uint32 i = 1; i < numValid; i++) {
    float ds2 = point1.distanceSquare(candidates[i]);

    if(ds2 > ds1) {
      i2 = i;
      ds1 = ds2;
    }
  }

  Vector2 point2 = candidates[i2];
  candidates[i2] = candidates[numValid--];

  Vector2 right[MAX_POLYGON_VERTICES - 2];
  uint32 numRight = 0;
  Vector2 left[MAX_POLYGON_VERTICES - 2];
  uint32 numLeft = 0;
  Vector2 divider = point2 - point1;
  divider.normalize();

  for(uint32 i = 0; i < numValid; i++) {
    float distance = (candidates[i] - point1).cross(divider);

    if(distance >= LINEAR_SLOP) {
      right[numRight++] = candidates[i];
    }
    else if(distance <= -LINEAR_SLOP) {
      left[numLeft++] = candidates[i];
    }
  }

  Hull rightHull = recurseHull(point1, point2, right, numRight);
  Hull leftHull = recurseHull(point2, point1, left, numLeft);

  /* Collinear */
  if(leftHull.numPoints == 0 && rightHull.numPoints == 0) {
    return hull;
  }

  hull.points[hull.numPoints++] = point1;

  for(uint32 i = 0; i < rightHull.numPoints; i++) {
    hull.points[hull.numPoints++] = rightHull.points[i];
  }

  hull.points[hull.numPoints++] = point2;

  for(uint32 i = 0; i < leftHull.numPoints; i++) {
    hull.points[hull.numPoints++] = leftHull.points[i];
  }

  assert(hull.numPoints <= MAX_POLYGON_VERTICES);

  /* Merge collinear points */
  bool valid = true;

  while(valid && hull.numPoints >= MIN_POLYGON_VERTICES) {
    valid = false;

    for(uint32 i = 0; i < hull.numPoints; i++) {
      Vector2 point1 = hull.points[i];
      Vector2 point2 = hull.points[(i + 1) % hull.numPoints];
      Vector2 point3 = hull.points[(i + 2) % hull.numPoints];
      Vector2 refLine = point3 - point1;
      refLine.normalize();
      float distance = (point2 - point1).cross(refLine);

      if(distance <= LINEAR_SLOP) {
        /* Remove midpoint of reference line */
        for(uint32 j = (i + 1) % hull.numPoints; j < hull.numPoints - 1; j++) {
          hull.points[j] = hull.points[j + 1];
        }

        hull.numPoints--;
        valid = true;
        break;
      }
    }
  }

  return hull;
}

Hull recurseHull(const Vector2& minPoint, const Vector2& maxPoint, Vector2* points, uint32 numPoints) {
  Hull hull;
  hull.numPoints = 0;

  if(numPoints == 0) {
    return hull;
  }

  Vector2 divider = maxPoint - minPoint;
  divider.normalize();
  Vector2 right[MAX_POLYGON_VERTICES];
  uint32 numRight = 0;
  uint32 furthestIndex = 0;
  float furthest = (points[furthestIndex] - minPoint).cross(divider);

  if(furthest > 0.0f) {
    right[numRight++] = points[furthestIndex];
  }

  for(uint32 i = 1; i < numPoints; i++) {
    float distance = (points[i] - minPoint).cross(divider);

    if(distance > furthest) {
      furthestIndex = i;
      furthest = distance;
    }

    if(distance > 0.0f) {
      right[numRight++] = points[i];
    }
  }

  if(furthest < LINEAR_SLOP) {
    return hull;
  }

  Vector2 furthestPoint = points[furthestIndex];

  Hull leftHull = recurseHull(minPoint, furthestPoint, right, numRight);
  Hull rightHull = recurseHull(furthestPoint, maxPoint, right, numRight);

  /* Stitch hulls together */
  for(uint32 i = 0; i < leftHull.numPoints; i++) {
    hull.points[hull.numPoints++] = leftHull.points[i];
  }

  hull.points[hull.numPoints++] = furthestPoint;

  for(uint32 i = 0; i < rightHull.numPoints; i++) {
    hull.points[hull.numPoints++] = rightHull.points[i];
  }

  assert(hull.numPoints < MAX_POLYGON_VERTICES);
  return hull;
}