#include "UnitTests.h"

#include <physics/Physics.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(World, General) {
  Logger* logger = new Logger(LOG_FILE);
  Factory factory;
  World* world;
  Body* bodyA;
  Transform transformA(Vector2(0.0f, -10.0f), Rotation(0.0f));
  Collider* colliderA;
  Body* bodyB;
  Transform transformB(Vector2(0.0f, 8.0f), Rotation(0.0f));
  Collider* colliderB;
  CircleShape* circle;
  BoxShape* box;
  Transform transformLocalBody;

  box = factory.createBox(50.0f, 10.0f);
  circle = factory.createCircle(1.0f);

  factory.setLogger(logger);
  world = factory.createWorld();

  bodyA = world->createBody(transformA);
  bodyA->setType(BodyType::Static);
  colliderA = bodyA->addCollider(box, transformLocalBody);
  bodyA->setMassPropertiesUsingColliders();
  
  bodyB = world->createBody(transformB);
  colliderB = bodyB->addCollider(circle, transformLocalBody);
  bodyB->setMassPropertiesUsingColliders();

  float timeStep = 1.0f / 60.0f;

  for(uint32 i = 0; i < 180; i++) {
    world->step(timeStep);
    const Transform& transform = bodyB->getTransform();
    const Vector2& position = transform.getPosition();
    const float angle = transform.getOrientation().getAngle();

    std::cout << "Dynamic Body Data: " << position.x << ", " << position.y << ", " << angle << std::endl;
  }
}