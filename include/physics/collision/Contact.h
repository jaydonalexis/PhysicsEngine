#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

#include <physics/collision/Collider.h>

namespace physics {

struct ContactFeature {

  public:
    /* -- Nested classes -- */
    enum class FeatureType {Face, Vertex};

    /* -- Attributes -- */

    /* Feature index on the first shape */
    uint8 firstIndex;

    /* Feature index on the second shape */
    uint8 secondIndex;

    /* Feature type on the first shape */
    FeatureType firstType;

    /* Feature type on the second shape */
    FeatureType secondType;
};

union ContactInfo {
  /* Key */
  uint32 key;

  /* Contact feature */
  ContactFeature feature;
};

struct ContactPoint {

  public:
    /* -- Attributes -- */

    /* Contact point info */
    ContactInfo info;

    /* Intrinsic local point */
    Vector2 localPoint;

    /* Non-penetration impulse */
    float normalImpulse;

    /* Friction impulse */
    float tangentImpulse;
};

struct ClipVertex {

  public:
    /* -- Attributes -- */

    /* Vertex */
    Vector2 vertex;

    /* Clip vertex info */
    ContactInfo info;
};

struct LocalManifoldInfo {

  public:
    /* -- Nested classes -- */
    enum class ManifoldType {Circles, FaceA, FaceB};

    /* -- Attributes -- */
    
    /* Manifold type */
    ManifoldType type;

    /* Local normal reference for contact maifold */
    Vector2 localNormal;

    /* Local point reference for contact manifold */
    Vector2 localPoint;

    /* Contact points owned by the manifold */
    ContactPoint points[MAX_MANIFOLD_POINTS];

    /* Number of points owned by the manifold */
    uint8 numPoints;
};

struct LocalManifold {

  public:
    /* -- Attributes -- */

    /* Manifold info */
    LocalManifoldInfo info;

    /* First body entity */
    Entity firstBodyEntity;

    /* Second body entity */
    Entity secondBodyEntity;

    /* First collider entity */
    Entity firstColliderEntity;

    /* Second collider entity */
    Entity secondColliderEntity;

  /* -- Methods -- */

  /* Constructor */
  LocalManifold(LocalManifoldInfo info,
                Entity firstBodyEntity,
                Entity secondBodyEntity,
                Entity firstColliderEntity,
                Entity secondColliderEntity);
};

struct WorldManifold {

  public:
    /* -- Attributes -- */
    
    /* World normal */
    Vector2 normal;

    /* World contact points */
    Vector2 points[MAX_MANIFOLD_POINTS];

    /* Contact point separations */
    float separations[MAX_MANIFOLD_POINTS];

    /* -- Methods -- */

    /* Constructor */
    WorldManifold(const LocalManifold& localManifold, Transform transformA, float radiusA, Transform transformB, float radiusB);
};

struct ContactPair {

  public:
    /* -- Attributes -- */

    /* Overlap pair identifier */
    uint64 overlapPairIdentifier;

    /* Contact pair index */
    uint32 contactPairIndex;

    /* Index in the array of raw contact manifolds */
    uint32 rawManifoldsIndex;

    /* Index in the array of island packed manifolds */
    uint32 manifoldsIndex;

    /* First body entity */
    Entity firstBodyEntity;

    /* Second body entity */
    Entity secondBodyEntity;

    /* First collider entity */
    Entity firstColliderEntity;

    /* Second collider entity */
    Entity secondColliderEntity;

    /* Manifold is already in an island */
    bool isInIsland;

    /* -- Methods -- */

    /* Constructor */
    ContactPair(uint64 overlapPairIdentifier,
                uint32 contactPairIndex,
                Entity firstBodyEntity,
                Entity secondBodyEntity,
                Entity firstColliderEntity,
                Entity secondColliderEntity) :
                overlapPairIdentifier(overlapPairIdentifier),
                contactPairIndex(contactPairIndex),
                firstBodyEntity(firstBodyEntity),
                secondBodyEntity(secondBodyEntity),
                firstColliderEntity(firstColliderEntity),
                secondColliderEntity(secondColliderEntity),
                isInIsland(false) {}
};

inline LocalManifold::LocalManifold(LocalManifoldInfo info,
                                    Entity firstBodyEntity,
                                    Entity secondBodyEntity,
                                    Entity firstColliderEntity,
                                    Entity secondColliderEntity) :
                                    info(info),
                                    firstBodyEntity(firstBodyEntity),
                                    secondBodyEntity(secondBodyEntity),
                                    firstColliderEntity(firstColliderEntity),
                                    secondColliderEntity(secondColliderEntity) {}

inline WorldManifold::WorldManifold(const LocalManifold& localManifold, Transform transformA, float radiusA, Transform transformB, float radiusB) {
  if(!localManifold.info.numPoints) {
    return;
  }

  switch(localManifold.info.type) {
    case LocalManifoldInfo::ManifoldType::Circles:
    {
      normal.set(1.0f, 0.0f);
      Vector2 pointA = transformA * localManifold.info.localPoint;
      Vector2 pointB = transformB * localManifold.info.points[0].localPoint;
      float distanceSquared = pointA.distanceSquare(pointB);

      /* Debug */
      if(distanceSquared > square(FLOAT_EPSILON)) {
        normal = pointB - pointA;
        normal.normalize();
      }

      Vector2 pointC = pointA + radiusA * normal;
      Vector2 pointD = pointB - radiusB * normal;
      points[0] = 0.5f * (pointC + pointD);
      separations[0] = dot(pointD - pointC, normal);
    }
    
    break;

    case LocalManifoldInfo::ManifoldType::FaceA:
    {
      normal = transformA.getOrientation() * localManifold.info.localNormal;
      Vector2 planePoint = transformA * localManifold.info.localPoint;

      for(uint32 i = 0; i < localManifold.info.numPoints; i++) {
        Vector2 clipPoint = transformB * localManifold.info.points[i].localPoint;
        Vector2 pointA = clipPoint + (radiusA - dot(clipPoint - planePoint, normal)) * normal;
        Vector2 pointB = clipPoint - radiusB * normal;
        points[i] = 0.5f * (pointA + pointB);
        separations[i] = dot(pointB - pointA, normal);
      }
    }
    
    break;

    case LocalManifoldInfo::ManifoldType::FaceB:
    {
      normal = transformB.getOrientation() * localManifold.info.localNormal;
      Vector2 planePoint = transformB * localManifold.info.localPoint;

      for(uint32 i = 0; i < localManifold.info.numPoints; i++) {
        Vector2 clipPoint = transformA * localManifold.info.points[i].localPoint;
        Vector2 pointA = clipPoint - radiusA * normal;
        Vector2 pointB = clipPoint + (radiusB - dot(clipPoint - planePoint, normal)) * normal;
        points[i] = 0.5f * (pointA + pointB);
        separations[i] = dot(pointA - pointB, normal);
      }

      normal = -normal;
    }
    
    break;
  }
}

}

#endif