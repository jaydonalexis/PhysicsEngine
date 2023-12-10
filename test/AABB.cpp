#include "UnitTests.h"

#include <physics/Physics.h>
#include <physics/collision/AABB.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(AABB, Constructor)
{
    const auto lowerBound = Vector2{0.0f, 0.0f};
    const auto upperBound = Vector2{0.0f, 0.0f};
    const auto aabb = AABB{};
    EXPECT_EQ(aabb.getlowerBound(), lowerBound);
    EXPECT_EQ(aabb.getUpperBound(), upperBound);
}