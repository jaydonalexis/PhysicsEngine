#include "UnitTests.h"

#include <physics/collections/Queue.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Queue, Constructors) {
  VanillaMemoryHandler memoryHandler;
  Queue<std::string> queue1(memoryHandler);
  EXPECT_TRUE(queue1.size() == 0);
  EXPECT_TRUE(queue1.empty() == true);

  Queue<std::string> queue2(queue1);
  EXPECT_TRUE(queue1.size() == queue2.size());

  Queue<int> queue3(memoryHandler);
  queue3.push(10);
  queue3.push(20);
  queue3.push(30);
  EXPECT_TRUE(queue3.empty() == false);
  EXPECT_TRUE(queue3.size() == 3);

  queue3.push(40);
  EXPECT_TRUE(queue3.size() == 4);

  Queue<int> queue4(queue3);
  EXPECT_TRUE(queue4.size() == queue3.size());
  EXPECT_TRUE(queue4.pop() == 10);
  EXPECT_TRUE(queue4.size() == 3);
  EXPECT_TRUE(queue4.pop() == 20);
  EXPECT_TRUE(queue4.pop() == 30);
  EXPECT_TRUE(queue4.pop() == 40);
  EXPECT_TRUE(queue4.size() == 0);
  EXPECT_TRUE(queue4.empty() == true);
}

TEST(Queue, PushPop) {
  VanillaMemoryHandler memoryHandler;
  Queue<int> queue1(memoryHandler);
  queue1.push(10);
  queue1.push(80);
  queue1.push(130);
  EXPECT_TRUE(queue1.size() == 3);
  EXPECT_TRUE(queue1.pop() == 10);
  EXPECT_TRUE(queue1.pop() == 80);
  EXPECT_TRUE(queue1.pop() == 130);
  EXPECT_TRUE(queue1.size() == 0);

  queue1.push(10);
  EXPECT_TRUE(queue1.pop() == 10);

  queue1.push(10);
  queue1.push(80);
  queue1.pop();
  EXPECT_TRUE(queue1.pop() == 80);
  EXPECT_TRUE(queue1.size() == 0);

  queue1.push(10);
  queue1.push(80);
  queue1.push(130);
  queue1.push(56);
  queue1.push(89);
  queue1.push(131);
  queue1.clear();
  EXPECT_TRUE(queue1.size() == 0);
}