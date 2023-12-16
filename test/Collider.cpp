#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(Collider, GetSet) {
  Factory factory;
  World* world;
  Body* body;
  Collider* collider;
  BoxShape* box;
  CircleShape* circle;
  PolygonShape* polygon;
  const Transform transform1;
  const Transform transform2({2.0f, 1.0f}, {1.0f, 0.0f});
  box = factory.createBox(1.0f, 1.0f);
  circle = factory.createCircle(1.0f);
  Vector2 points[] = {{-2.0f, 0.0f}, {0.0f, -2.0f}, {2.0f, 0.0f}, {0.0f, 2.0f}};
  polygon = factory.createPolygon(points, 4);

  world = factory.createWorld();
  body = world->createBody(transform1);

  collider = body->addCollider(box, transform1);
  EXPECT_EQ(collider->getShape()->getType(), ShapeType::Polygon);
  EXPECT_EQ(collider->getEntity().getIndex(), 1);
  EXPECT_EQ(collider->getEntity().getGeneration(), 0);

  body->removeCollider(collider);

  collider = body->addCollider(circle, transform1);
  EXPECT_EQ(collider->getShape()->getType(), ShapeType::Circle);
  EXPECT_EQ(collider->getEntity().getIndex(), 2);
  EXPECT_EQ(collider->getEntity().getGeneration(), 0);

  body->removeCollider(collider);

  collider = body->addCollider(polygon, transform1);
  EXPECT_EQ(collider->getShape()->getType(), ShapeType::Polygon);
  EXPECT_EQ(collider->getEntity().getIndex(), 3);
  EXPECT_EQ(collider->getEntity().getGeneration(), 0);

  body->removeCollider(collider);

  collider = body->addCollider(box, transform1);
  EXPECT_EQ(collider->getBody(), body);

  EXPECT_EQ(collider->getTransformLocalBody(), transform1);

  collider->setTransformLocalBody(transform2);
  EXPECT_EQ(collider->getTransformLocalBody(), transform2);

  collider->setTransformLocalBody(transform1);
  body->setTransform(transform2);
  EXPECT_EQ(collider->getTransformLocalBody(), transform1);
  EXPECT_EQ(collider->getTransformLocalWorld(), transform2);

  body->removeCollider(collider);
  body->setTransform(transform1);

  collider = body->addCollider(polygon, transform1);
  AABB aabb = collider->getAABB();
  EXPECT_EQ(aabb.getlowerBound(), Vector2(-2.0f, -2.0f) - Vector2(POLYGON_RADIUS, POLYGON_RADIUS));
  EXPECT_EQ(aabb.getUpperBound(), Vector2(2.0f, 2.0f) + Vector2(POLYGON_RADIUS, POLYGON_RADIUS));

  EXPECT_TRUE(collider->testOverlap(AABB({1.02f, -2.0f}, {2.02f, 0.0f})));
  EXPECT_FALSE(collider->testOverlap(AABB({2.02f, -0.5f}, {3.52f, 0.5f})));

  EXPECT_TRUE(collider->testPoint({0.0f, 0.0f}));
  EXPECT_TRUE(collider->testPoint({2.00f, 0.0f}));
  EXPECT_FALSE(collider->testPoint({2.01f, 0.0f}));


  enum CollisionCategory {
    CATEGORY1 = 0x0001,
    CATEGORY2 = 0x0002,
    CATEGORY3 = 0x0004
  };

  collider->setCollisionCategory(CATEGORY1);
  EXPECT_EQ(collider->getCollisionCategory(), CATEGORY1);

  collider->setCollisionFilter(CATEGORY1 | CATEGORY2);
  EXPECT_EQ(collider->getCollisionFilter(), CATEGORY1 | CATEGORY2);

  body->removeCollider(collider);

  collider = body->addCollider(circle, transform1);
  EXPECT_EQ(collider->getBroadPhaseIdentifier(), 0);

  collider->getMaterial().setFriction(0.3f);
  collider->getMaterial().setRestitution(0.4f);
  collider->getMaterial().setDensity(0.5f);
  EXPECT_EQ(collider->getMaterial().getFriction(), 0.3f);
  EXPECT_EQ(collider->getMaterial().getRestitution(), 0.4f);
  EXPECT_EQ(collider->getMaterial().getDensity(), 0.5f);
}