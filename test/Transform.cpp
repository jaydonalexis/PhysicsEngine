#include "UnitTests.h"

#include <physics/mathematics/Transform.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Transform, Constructor) {
  Transform transform1;
  EXPECT_TRUE(transform1.getPosition() == Vector2(0, 0));
  EXPECT_TRUE(transform1.getOrientation() == Rotation::getZero());

  Transform transform2(Vector2(1, 2), Rotation(0.5f));
  EXPECT_TRUE(transform2.getPosition() == Vector2(1, 2));
  EXPECT_TRUE(approximateEqual(transform2.getOrientation().s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(transform2.getOrientation().c, cosf(0.5f)));
}

TEST(Transform, CopyConstructor) {
  Transform transform1(Vector2(1, 2), Rotation(0.5f));
  Transform transform2(transform1);
  EXPECT_TRUE(transform2.getPosition() == Vector2(1, 2));
  EXPECT_TRUE(approximateEqual(transform2.getOrientation().s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(transform2.getOrientation().c, cosf(0.5f)));
}

TEST(Transform, SetAndGetPosition) {
  Transform transform;
  transform.setPosition(Vector2(1, 2));
  EXPECT_TRUE(transform.getPosition() == Vector2(1, 2));
}

TEST(Transform, SetAndGetOrientation) {
  Transform transform;
  transform.setOrientation(Rotation(0.5f));
  EXPECT_TRUE(approximateEqual(transform.getOrientation().s, sinf(0.5f)));
  EXPECT_TRUE(approximateEqual(transform.getOrientation().c, cosf(0.5f)));
}

TEST(Transform, Operators) {
  EXPECT_TRUE(Transform(Vector2(0, 1), Rotation(0.5f)) == Transform(Vector2(0, 1), Rotation(0.5f)));
  EXPECT_TRUE(Transform(Vector2(1, 0), Rotation(0.5f)) != Transform(Vector2(0, 1), Rotation(0.5f)));
  EXPECT_TRUE(Transform(Vector2(0, 1), Rotation(0.25f)) != Transform(Vector2(0, 1), Rotation(0.5f)));
  EXPECT_TRUE(Transform(Vector2(1, 0), Rotation(0.25f)) != Transform(Vector2(0, 1), Rotation(0.5f)));
  
  Transform transform1(Vector2(1, 2), Rotation(0.5f));
  Transform transform2(Vector2(1, 2), Rotation(0.25f));
  Transform transform3(Vector2(3, 4), Rotation(0.5f));
  Transform transform4(Vector2(1, 2), Rotation(0.5f));
  EXPECT_TRUE(transform1 != transform2);
  EXPECT_TRUE(transform1 != transform3);
  EXPECT_TRUE(transform1 == transform4);

  transform3 = transform1 * transform2;
  EXPECT_TRUE(transform3.getPosition() == (transform1.getOrientation() * transform2.getPosition()) + transform1.getPosition());
  EXPECT_TRUE(transform3.getOrientation() == transform1.getOrientation() * transform2.getOrientation());

  transform4 = transform1;
  transform1 *= transform2;
  EXPECT_TRUE(transform1.getPosition() == (transform4.getOrientation() * transform2.getPosition()) + transform4.getPosition());
  EXPECT_TRUE(transform1.getOrientation() == transform4.getOrientation() * transform2.getOrientation());

  Transform transform5(Vector2(1, 2), Rotation(PI / 8.0f));
  Vector2 vector1{4, 3};
  Vector2 vector2;
  vector2 = transform5 * vector1;
  EXPECT_TRUE(vector2 == Vector2(3.5474678f, 6.3023723f));
}

TEST(Transform, SetIdentity) {
  Transform transform(Vector2(1, 2), Rotation(0.5f));
  transform.setIdentity();
  EXPECT_TRUE(transform.getPosition() == Vector2(0, 0));
  EXPECT_TRUE(transform.getOrientation() == Rotation::getZero());
}

TEST(Transform, GetIdentity) {
  Transform transform1(Vector2(1, 2), Rotation(0.5f));
  Transform transform2 = transform1.getIdentity();
  EXPECT_TRUE(transform2.getPosition() == Vector2(0, 0));
  EXPECT_TRUE(transform2.getOrientation() == Rotation::getZero());
}

TEST(Transform, InverseMultiply) {
  Transform transform1(Vector2(1, 2), Rotation(PI / 8.0f));
  Transform transform2(Vector2(3, 4), Rotation{PI / 4.0f});
  Transform transform3 = transform1 ^ transform2;
  EXPECT_TRUE(transform3.getPosition() == (transform1.getOrientation() ^ transform2.getPosition() - transform1.getPosition()));
  EXPECT_TRUE(transform3.getOrientation() == (transform1.getOrientation() ^ transform2.getOrientation()));

  Transform transform4(Vector2(1, 2), Rotation(PI / 8.0f));
  Vector2 vector1{4, 3};
  Vector2 vector2;
  vector2 = transform4 ^ vector1;
  EXPECT_TRUE(vector2 == Vector2(3.1543220f, -0.2241707f));
}