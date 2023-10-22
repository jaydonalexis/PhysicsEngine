#include "UnitTests.h"

#include <physics/collections/Stack.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(Stack, Constructors) {
  VanillaMemoryHandler memoryHandler;
  Stack<std::string> stack1(memoryHandler);
  EXPECT_TRUE(stack1.size() == 0);
  EXPECT_TRUE(stack1.empty() == true);

  Stack<std::string> stack2(stack1);
  EXPECT_TRUE(stack1.size() == stack2.size());

  Stack<int> stack3(memoryHandler);
  stack3.push(10);
  stack3.push(20);
  stack3.push(30);
  EXPECT_TRUE(stack3.empty() == false);
  EXPECT_TRUE(stack3.size() == 3);

  stack3.push(40);
  EXPECT_TRUE(stack3.size() == 4);

  Stack<int> stack4(stack3);
  EXPECT_TRUE(stack4.size() == stack3.size());
  EXPECT_TRUE(stack4.pop() == 40);
  EXPECT_TRUE(stack4.size() == 3);
  EXPECT_TRUE(stack4.pop() == 30);
  EXPECT_TRUE(stack4.pop() == 20);
  EXPECT_TRUE(stack4.pop() == 10);
  EXPECT_TRUE(stack4.size() == 0);
  EXPECT_TRUE(stack4.empty() == true);
}

TEST(Stack, PushPop) {
  VanillaMemoryHandler memoryHandler;
  Stack<int> stack1(memoryHandler);
  stack1.push(10);
  stack1.push(80);
  stack1.push(130);
  EXPECT_TRUE(stack1.size() == 3);
  EXPECT_TRUE(stack1.pop() == 130);
  EXPECT_TRUE(stack1.pop() == 80);
  EXPECT_TRUE(stack1.pop() == 10);
  EXPECT_TRUE(stack1.size() == 0);

  stack1.push(10);
  EXPECT_TRUE(stack1.pop() == 10);

  stack1.push(10);
  stack1.push(80);
  stack1.pop();
  EXPECT_TRUE(stack1.pop() == 10);
  EXPECT_TRUE(stack1.size() == 0);

  stack1.push(10);
  stack1.push(80);
  stack1.push(130);
  stack1.push(56);
  stack1.push(89);
  stack1.push(131);
  stack1.clear();
  EXPECT_TRUE(stack1.size() == 0);
}