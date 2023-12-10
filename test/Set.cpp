#include "UnitTests.h"

#include <physics/collections/Set.h>
#include <physics/collections/DynamicArray.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

namespace physics {
  struct TestValue {
    int value;

    TestValue(int v) : value(v) {}

    bool operator==(const TestValue& testValue) const {
      return value == testValue.value;
    }
  };
}

namespace std {
  template <> struct hash<physics::TestValue> {
    size_t operator()(const physics::TestValue&) const {
      return 1;
    }
  };
}

TEST(Set, Constructors) {
  VanillaMemoryHandler memoryHandler;
  Set<std::string> set1(memoryHandler);
  EXPECT_TRUE(set1.size() == 0);

  Set<std::string> set2(memoryHandler);
  EXPECT_TRUE(set2.size() == 0);

  Set<std::string> set3(set1);
  EXPECT_TRUE(set3.size() == set1.size());

  Set<int> set4(memoryHandler);
  set4.insert(10);
  set4.insert(20);
  set4.insert(30);
  EXPECT_TRUE(set4.size() == 3);

  set4.insert(30);
  EXPECT_TRUE(set4.size() == 3);

  Set<int> set5(set4);
  EXPECT_TRUE(set5.size() == set4.size());
  EXPECT_TRUE(set5.contains(10) == true);
  EXPECT_TRUE(set5.contains(20) == true);
  EXPECT_TRUE(set5.contains(30) == true);
}

TEST(Set, Modifiers) {
  VanillaMemoryHandler memoryHandler;
  Set<int> set1(memoryHandler);
  set1.insert(10);
  set1.insert(80);
  set1.insert(130);
  EXPECT_TRUE(set1.contains(10) == true);
  EXPECT_TRUE(set1.contains(80) == true);
  EXPECT_TRUE(set1.contains(130) == true);
  EXPECT_TRUE(set1.size() == 3);

  set1.insert(80);
  EXPECT_TRUE(set1.contains(80) == true);
  EXPECT_TRUE(set1.size() == 3);

  Set<int> set2(memoryHandler);

  for(int i = 0; i < 1000000; i++) {
    set2.insert(i);
  }

  bool valid = true;

  for(int i = 0; i < 1000000; i++) {
    if (!set2.contains(i)) {
      valid = false;
    }
  }

  EXPECT_TRUE(valid == true);

  set1.remove(10);
  set1.insert(10);
  EXPECT_TRUE(set1.size() == 3);
  EXPECT_TRUE(set1.contains(10) == true);

  set1.insert(34);
  EXPECT_TRUE(set1.contains(34) == true);
  EXPECT_TRUE(set1.size() == 4);

  set1.remove(10);
  EXPECT_TRUE(!set1.contains(10) == true);
  EXPECT_TRUE(set1.contains(80) == true);
  EXPECT_TRUE(set1.contains(130) == true);
  EXPECT_TRUE(set1.contains(34) == true);
  EXPECT_TRUE(set1.size() == 3);

  set1.remove(80);
  EXPECT_TRUE(!set1.contains(80) == true);
  EXPECT_TRUE(set1.contains(130) == true);
  EXPECT_TRUE(set1.contains(34) == true);
  EXPECT_TRUE(set1.size() == 2);

  set1.remove(130);
  EXPECT_TRUE(!set1.contains(130) == true);
  EXPECT_TRUE(set1.contains(34) == true);
  EXPECT_TRUE(set1.size() == 1);

  set1.remove(34);
  EXPECT_TRUE(!set1.contains(34) == true);
  EXPECT_TRUE(set1.size() == 0);

  valid = true;

  for(int i = 0; i < 1000000; i++) {
    set2.remove(i);
  }

  for(int i = 0; i < 1000000; i++) {
    if (set2.contains(i)) valid = false;
  }

  EXPECT_TRUE(valid == true);
  EXPECT_TRUE(set2.size() == 0);

  Set<int> set3(memoryHandler);

  for(int i=0; i < 1000000; i++) {
    set3.insert(i);
    set3.remove(i);
  }

  set3.insert(1);
  set3.insert(2);
  set3.insert(3);
  EXPECT_TRUE(set3.size() == 3);

  auto it = set3.begin();
  it = set3.remove(it);
  EXPECT_TRUE(set3.size() == 2);

  it = set3.remove(it);
  EXPECT_TRUE(set3.size() == 1);

  it = set3.remove(it);
  EXPECT_TRUE(set3.size() == 0);

  set3.insert(6);
  set3.insert(7);
  set3.insert(8);

  for(it = set3.begin(); it != set3.end();) {
    it = set3.remove(it);
  }

  EXPECT_TRUE(set3.size() == 0);

  Set<int> set4(memoryHandler);
  set4.insert(2);
  set4.insert(4);
  set4.insert(6);
  set4.clear();
  EXPECT_TRUE(set4.size() == 0);

  set4.insert(2);
  EXPECT_TRUE(set4.size() == 1);
  EXPECT_TRUE(set4.contains(2) == true);

  set4.clear();
  EXPECT_TRUE(set4.size() == 0);

  Set<int> set5(memoryHandler);
  set5.clear();
  EXPECT_TRUE(set5.size() == 0);

  Set<TestValue> set6(memoryHandler);

  for(int i=0; i < 1000; i++) {
    set6.insert(TestValue(i));
  }

  valid = true;

  for(int i=0; i < 1000; i++) {
    if (!set6.contains(TestValue(i))) {
      valid = false;
    }
  }

  EXPECT_TRUE(valid == true);

  for(int i=0; i < 1000; i++) {
    set6.remove(TestValue(i));
  }

  EXPECT_TRUE(set6.size() == 0);
}

TEST(Set, ToArray) {
  VanillaMemoryHandler memoryHandler;
  Set<int> set1(memoryHandler);
  set1.insert(1);
  set1.insert(2);
  set1.insert(3);
  set1.insert(4);
  DynamicArray<int> array1 = set1.toArray(memoryHandler);
  EXPECT_TRUE(array1.size() == 4);
  EXPECT_TRUE(array1.find(1) != array1.end());
  EXPECT_TRUE(array1.find(2) != array1.end());
  EXPECT_TRUE(array1.find(3) != array1.end());
  EXPECT_TRUE(array1.find(4) != array1.end());
  EXPECT_TRUE(array1.find(5) == array1.end());
  EXPECT_TRUE(array1.find(6) == array1.end());

  Set<int> set2(memoryHandler);
  DynamicArray<int> array2 = set2.toArray(memoryHandler);
  EXPECT_TRUE(array2.size() == 0);
}