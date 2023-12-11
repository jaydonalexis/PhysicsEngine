#ifndef PHYSICS_CONVEX_HULL_H
#define PHYSICS_CONVEX_HULL_H

#include <physics/Configuration.h>
#include <physics/mathematics/Math.h>
#include <physics/collision/AABB.h>
#include <cassert>
#include <iostream>

namespace physics {

struct Hull {

  public:
    /* -- Attributes -- */

    /* Hull points */
    Vector2 points[MAX_POLYGON_VERTICES];

    /* Number of hull points */
    uint32 numPoints;

    /* -- Methods -- */
    
    /* Constructor */
    Hull() : numPoints(0) {}
};

/* Recursively compute convex hull */
static inline Hull computeHull(const Vector2& minPoint, const Vector2& maxPoint, const Vector2* points, uint32 numPoints) {
  Hull hull;

  if(numPoints == 0) {
    return hull;
  }

  /* Edge vector pointing from the minimum point to the maximum point in the current reference frame */
  Vector2 divider = maxPoint - minPoint;
  divider.normalize();

  /* Consider points only to the right of this edge vector */
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

  /* Hull computed using the points to the right of the line connecting the minimum point to the furthest point from the edge vector in the current reference frame */
  Hull rightHull = computeHull(minPoint, furthestPoint, right, numRight);

  /* Hull computed using the points to the right of the line connecting the furthest point from edge vector to the maximum point in the current reference frame */
  Hull leftHull = computeHull(furthestPoint, maxPoint, right, numRight);

  /* Stitch hulls together */
  for(uint32 i = 0; i < rightHull.numPoints; i++) {
    hull.points[hull.numPoints++] = rightHull.points[i];
  }

  hull.points[hull.numPoints++] = furthestPoint;

  for(uint32 i = 0; i < leftHull.numPoints; i++) {
    hull.points[hull.numPoints++] = leftHull.points[i];
  }

  assert(hull.numPoints < MAX_POLYGON_VERTICES);
  return hull;
}

/* Create a convex hull from the provided set of points */
inline Hull getHull(const Vector2* points, uint32 numPoints) {
  Hull hull;
  hull.numPoints = 0;

  if(numPoints < MIN_POLYGON_VERTICES || numPoints > MAX_POLYGON_VERTICES) {
    return hull;
  }

  numPoints = std::min(numPoints, (uint32)MAX_POLYGON_VERTICES);
  AABB aabb{{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
  Vector2 candidates[MAX_POLYGON_VERTICES];
  uint32 numValid = 0;

  /* Merge points that are in close proximity to each other */
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

  /* Remove this first point from the current set of points */
  Vector2 point1 = candidates[i1];
  candidates[i1] = candidates[numValid - 1];
  numValid--;

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

  /* Remove this second point from the current set of points */
  Vector2 point2 = candidates[i2];
  candidates[i2] = candidates[numValid - 1];
  numValid--;

  Vector2 right[MAX_POLYGON_VERTICES - 2];
  uint32 numRight = 0;
  Vector2 left[MAX_POLYGON_VERTICES - 2];
  uint32 numLeft = 0;
  Vector2 divider = point2 - point1;
  divider.normalize();

  for(uint32 i = 0; i < numValid; i++) {
    float distance = cross(candidates[i] - point1, divider);

    /* Disregard points that are in close proximity to the line connecting the first and second extreme points from above */
    if(distance >= 2.0f * LINEAR_SLOP) {
      right[numRight++] = candidates[i];
    }
    else if(distance <= -2.0f * LINEAR_SLOP) {
      left[numLeft++] = candidates[i];
    }
  }

  /* Divide and conquer using a set of points to the right and left of the line connecting the first and second extreme points from above */
  Hull rightHull = computeHull(point1, point2, right, numRight);
  Hull leftHull = computeHull(point2, point1, left, numLeft);

  /* All of the points are approximately collinear */
  if(rightHull.numPoints == 0 && leftHull.numPoints == 0) {
    return hull;
  }

  /* Stitch hulls together */
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
      Vector2 pointA = hull.points[i];
      Vector2 pointB = hull.points[(i + 1) % hull.numPoints];
      Vector2 pointC = hull.points[(i + 2) % hull.numPoints];
      Vector2 refLine = pointC - pointA;
      refLine.normalize();
      float distance = cross(pointB - pointA, refLine);

      if(distance <= 2.0f * LINEAR_SLOP) {
        /* Remove midpoint of reference line */
        for(uint32 j = (i + 1) % hull.numPoints; j < hull.numPoints - 1; j++) {
          hull.points[j] = hull.points[j + 1];
        }

        hull.numPoints--;
        /* Collinear points may still exist */
        valid = true;
        break;
      }
    }
  }

  assert(hull.numPoints >= MIN_POLYGON_VERTICES);
  return hull;
}

}

#endif