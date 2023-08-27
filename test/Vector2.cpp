#include "UnitTests.h"

#include <physics/mathematics/Vector2.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Vector2, Constructor)
{
    Vector2 vector{5, 3};
    EXPECT_EQ(vector.x, 5.0);
    EXPECT_EQ(vector.y, 3.0);
}
