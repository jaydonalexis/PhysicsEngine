#ifndef PHYSICS_CONVEX_HULL_H
#define PHYSICS_CONVEX_HULL_H

#include <physics/Configuration.h>
#include <physics/mathematics/Math.h>
#include <physics/collision/AABB.h>
#include <cassert>

namespace physics {

class Hull {

  private:
    /* -- Attributes -- */

    /* Hull points */
    Vector2 mPoints[MAX_POLYGON_VERTICES];

    /* Number of hull points */
    uint32 mNumPoints;

  public:
    /* -- Methods -- */

    /* Constructor */
    Hull() = default;

    /* Constructor */
    Hull(const Vector2* points, uint32 numPoints);

    /* Initialize the convex hull */
    void init(const Vector2* points, uint32 numpoints);

    /* Recursive divide and conquer quick hull algorithm implementation */
    Hull recurse(const Vector2& minPoint, const Vector2& maxPoint, const Vector2* points, uint32 numPoints);

    /* -- Friends -- */
    friend class PolygonShape;
};

/* Constructor */
inline Hull::Hull(const Vector2* points, uint32 numPoints) {
  init(points, numPoints);
}

inline /* Initialize the convex hull */
void Hull::init(const Vector2* points, uint32 numPoints) {
  if(numPoints < MIN_POLYGON_VERTICES || numPoints > MAX_POLYGON_VERTICES) {
    return;
  }

  numPoints = std::min(numPoints, (uint32)MAX_POLYGON_VERTICES);
  AABB aabb{{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
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
    return;
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
    float distance = cross(candidates[i] - point1, divider);

    if(distance >= 2.0f * LINEAR_SLOP) {
      right[numRight++] = candidates[i];
    }
    else if(distance <= -2.0f * LINEAR_SLOP) {
      left[numLeft++] = candidates[i];
    }
  }

  Hull rightHull = recurse(point1, point2, right, numRight);
  Hull leftHull = recurse(point2, point1, left, numLeft);
  assert(leftHull.mNumPoints > 0 || rightHull.mNumPoints > 0);

  /* Collinear */
  if(leftHull.mNumPoints == 0 && rightHull.mNumPoints == 0) {
    return;
  }

  mPoints[mNumPoints++] = point1;

  for(uint32 i = 0; i < rightHull.mNumPoints; i++) {
    mPoints[mNumPoints++] = rightHull.mPoints[i];
  }

  mPoints[mNumPoints++] = point2;

  for(uint32 i = 0; i < leftHull.mNumPoints; i++) {
    mPoints[mNumPoints++] = leftHull.mPoints[i];
  }

  assert(mNumPoints <= MAX_POLYGON_VERTICES);

  /* Merge collinear points */
  bool valid = true;

  while(valid && mNumPoints >= MIN_POLYGON_VERTICES) {
    valid = false;

    for(uint32 i = 0; i < mNumPoints; i++) {
      Vector2 pointA = mPoints[i];
      Vector2 pointB = mPoints[(i + 1) % mNumPoints];
      Vector2 pointC = mPoints[(i + 2) % mNumPoints];
      Vector2 refLine = pointC - pointA;
      refLine.normalize();
      float distance = cross(pointB - pointA, refLine);

      if(distance <= 2.0f * LINEAR_SLOP) {
        /* Remove midpoint of reference line */
        for(uint32 j = (i + 1) % mNumPoints; j < mNumPoints - 1; j++) {
          mPoints[j] = mPoints[j + 1];
        }

        mNumPoints--;
        valid = true;
        break;
      }
    }
  }
}

/* Recursive divide and conquer quick hull algorithm implementation */
inline Hull Hull::recurse(const Vector2& minPoint, const Vector2& maxPoint, const Vector2* points, uint32 numPoints) {
  Hull hull;
  assert(hull.mNumPoints == 0);

  if(numPoints == 0) {
    return hull;
  }

  Vector2 divider = maxPoint - minPoint;
  divider.normalize();
  Vector2 right[MAX_POLYGON_VERTICES];
  uint32 numRight = 0;
  uint32 furthestIndex = 0;
  float furthest = cross(points[furthestIndex] - minPoint, divider);

  if(furthest > 0.0f) {
    right[numRight++] = points[furthestIndex];
  }

  for(uint32 i = 1; i < numPoints; i++) {
    float distance = cross(points[i] - minPoint, divider);

    if(distance > furthest) {
      furthestIndex = i;
      furthest = distance;
    }

    if(distance > 0.0f) {
      right[numRight++] = points[i];
    }
  }

  if(furthest < 2.0f * LINEAR_SLOP) {
    return hull;
  }

  Vector2 furthestPoint = points[furthestIndex];

  Hull rightHull = recurse(minPoint, furthestPoint, right, numRight);
  Hull leftHull = recurse(furthestPoint, maxPoint, right, numRight);

  /* Stitch hulls together */
  for(uint32 i = 0; i < rightHull.mNumPoints; i++) {
    hull.mPoints[hull.mNumPoints++] = rightHull.mPoints[i];
  }

  hull.mPoints[hull.mNumPoints++] = furthestPoint;

  for(uint32 i = 0; i < leftHull.mNumPoints; i++) {
    hull.mPoints[hull.mNumPoints++] = leftHull.mPoints[i];
  }

  assert(hull.mNumPoints < MAX_POLYGON_VERTICES);
  return hull;
}

}

#endif