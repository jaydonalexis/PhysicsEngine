#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(Body, GetSet) {
  Factory factory;
  World* world;
  Body* body;
  Body* circleBody;
  Body* boxBody;
  Collider* circleCollider;
  Collider* boxCollider;

  world = factory.createWorld();
  
  const Transform transform1(Vector2(1.0f, 2.0f), Rotation(0.0f));
  body = world->createBody(transform1);

  EXPECT_EQ(body->getEntity().getIndex(), 0);
  EXPECT_EQ(body->getEntity().getGeneration(), 0);

  const Transform transform2(Vector2(0.0f, 0.0f), Rotation(0.0f));
  boxBody = world->createBody(transform2);
  BoxShape* boxShape = factory.createBox(2.0f, 2.0f);
  boxCollider = boxBody->addCollider(boxShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  circleBody = world->createBody(transform2);
  CircleShape* circleShape = factory.createCircle(4.0f);
  circleCollider = circleBody->addCollider(circleShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  body->setMass(56.0f);
  EXPECT_EQ(body->getMass(), 56.0f);

  body->setLinearDamping(0.9f);
  EXPECT_EQ(body->getLinearDamping(), 0.9f);

  body->setAngularDamping(0.8f);
  EXPECT_EQ(body->getAngularDamping(), 0.8f);

  body->setLinearVelocity(Vector2(2.0f, 2.0f));
  EXPECT_EQ(body->getLinearVelocity(), Vector2(2.0f, 2.0f));

  body->setAngularSpeed(0.7f);
  EXPECT_EQ(body->getAngularSpeed(), 0.7f);

  body->setTransform(Transform(Vector2(3.0f, 4.0f), Rotation(PI / 2)));
  EXPECT_EQ(body->getTransform().getPosition(), Vector2(3.0f, 4.0f));
  EXPECT_NEAR(body->getTransform().getOrientation().c, 0.0f, FLOAT_EPSILON);
  EXPECT_NEAR(body->getTransform().getOrientation().s, 1.0f, FLOAT_EPSILON);

  body->setCenterOfMassLocal(Vector2(5.0f, 6.0f));
  EXPECT_EQ(body->getCenterOfMassLocal(), Vector2(5.0f, 6.0f));

  body->setType(BodyType::Kinematic);
  EXPECT_EQ(body->getType(), BodyType::Kinematic);
}

TEST(Body, MassProperties) {
  Factory factory;
  World* world;
  Body* body;
  Body* circleBody;
  Body* boxBody;
  Collider* circleCollider;
  Collider* boxCollider;

  world = factory.createWorld();
  
  const Transform transform1(Vector2(1.0f, 2.0f), Rotation(0.0f));
  body = world->createBody(transform1);

  const Transform transform2(Vector2(0.0f, 0.0f), Rotation(0.0f));
  boxBody = world->createBody(transform2);
  BoxShape* boxShape = factory.createBox(2.0f, 2.0f);
  boxCollider = boxBody->addCollider(boxShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  circleBody = world->createBody(transform2);
  CircleShape* circleShape = factory.createCircle(4.0f);
  circleCollider = circleBody->addCollider(circleShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  boxCollider->getMaterial().setDensity(4.0f);
  boxBody->setMassPropertiesUsingColliders();
  EXPECT_EQ(boxBody->getMass(), 16.0f);

  boxBody->setCenterOfMassLocal(Vector2(1.0f, 2.0f));
  boxBody->setMass(2.0f);
  boxBody->setMassPropertiesUsingColliders();
  EXPECT_EQ(boxBody->getMass(), 16.0f);

  circleCollider->getMaterial().setDensity(3.0f);
  circleBody->setMassPropertiesUsingColliders();
  const float circleMass = PI * square(4.0f) * 3.0f;
  EXPECT_EQ(circleBody->getMass(), circleMass);

  circleBody->setCenterOfMassLocal(Vector2(2.0f, 3.0f));
  circleBody->setMass(2.0f);
  circleBody->setMassPropertiesUsingColliders();
  EXPECT_EQ(circleBody->getMass(), circleMass);
  EXPECT_EQ(circleBody->getCenterOfMassLocal(), Vector2(0.0f, 0.0f));
}

TEST(Body, Stimuli) {
  Factory factory;
  World* world;
  Body* body;
  Body* circleBody;
  Body* boxBody;
  Collider* circleCollider;
  Collider* boxCollider;

  world = factory.createWorld();
  
  const Transform transform1(Vector2(1.0f, 2.0f), Rotation(0.0f));
  body = world->createBody(transform1);

  const Transform transform2(Vector2(0.0f, 0.0f), Rotation(0.0f));
  boxBody = world->createBody(transform2);
  BoxShape* boxShape = factory.createBox(2.0f, 2.0f);
  boxCollider = boxBody->addCollider(boxShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  circleBody = world->createBody(transform2);
  CircleShape* circleShape = factory.createCircle(4.0f);
  circleCollider = circleBody->addCollider(circleShape, Transform(Vector2(0.0f, 0.0f), Rotation(0.0f)));

  const Transform& worldTransform = body->getTransform();
  const Rotation orientation = worldTransform.getOrientation();

  body->applyForceToCenter(Vector2(4.0f, 5.0f));
  EXPECT_EQ(body->getForce(), Vector2(4.0f, 5.0f));
  EXPECT_EQ(body->getTorque(), 0.0f);

  body->clearForces();
  body->clearTorques();

  body->applyForceToCenter(Vector2(2.0f, 3.0f));
  EXPECT_EQ(body->getForce(), Vector2(2.0f, 3.0f));
  EXPECT_EQ(body->getTorque(), 0.0f);

  body->clearForces();
  body->clearTorques();

  body->applyForce(orientation * Vector2(0.0f, 3.0f), worldTransform * Vector2(2.0f, 0.0f));
  EXPECT_EQ(body->getForce(), orientation * Vector2(0.0f, 3.0f));
  EXPECT_EQ(body->getTorque(), (orientation * Vector2(2.0f * 3.0f, 0.0f)).length());

  body->clearForces();
  body->clearTorques();

  body->applyTorque(0.4f);
  EXPECT_EQ(body->getForce(), Vector2(0.0f, 0.0f));
  EXPECT_EQ(body->getTorque(), 0.4f);
}
