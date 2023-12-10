#include "UnitTests.h"

#include <physics/collections/Map.h>
#include <physics/collections/Pair.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

namespace physics {
  struct TestKey {
    int key;

    TestKey(int k) : key(k) {}

    bool operator==(const TestKey& testKey) const {
      return key == testKey.key;
    }
  };
}

namespace std {
  template <> struct hash<physics::TestKey> {
    size_t operator()(const physics::TestKey&) const {
      return 1;
    }
  };
}

TEST(Map, Constructors) {
  VanillaMemoryHandler memoryHandler;
  Map<int, std::string> map1(memoryHandler);
  EXPECT_TRUE(map1.size() == 0);

  Map<int, std::string> map3(map1);
  EXPECT_TRUE(map3.size() == map1.size());

  Map<int, int> map4(memoryHandler);
  map4.insert(Pair<int, int>(10, 10));
  map4.insert(Pair<int, int>(20, 20));
  map4.insert(Pair<int, int>(30, 30));
  EXPECT_TRUE(map4.size() == 3);

  Map<int, int> map5(map4);
  EXPECT_TRUE(map5.size() == map4.size());
  EXPECT_TRUE(map5[10] == 10);
  EXPECT_TRUE(map5[20] == 20);
  EXPECT_TRUE(map5[30] == 30);
}

TEST(Map, Modifiers) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  map1.insert(Pair<int, int>(1, 10));
  map1.insert(Pair<int, int>(8, 80));
  map1.insert(Pair<int, int>(13, 130));
  EXPECT_TRUE(map1[1] == 10);
  EXPECT_TRUE(map1[8] == 80);
  EXPECT_TRUE(map1[13] == 130);
  EXPECT_TRUE(map1.size() == 3);

  Map<int, int> map2(memoryHandler);

  for(int i = 0; i < 1000000; i++) {
    map2.insert(Pair<int, int>(i, i * 100));
  }

  bool valid = true;

  for(int i = 0; i < 1000000; i++) {
    if(map2[i] != i * 100) {
      valid = false;
    }
  }

  EXPECT_TRUE(valid == true);
  EXPECT_TRUE(map2.size() == 1000000);

  map1.remove(1);
  map1.insert(Pair<int, int>(1, 10));
  EXPECT_TRUE(map1.size() == 3);
  EXPECT_TRUE(map1[1] == 10);

  map1.insert(Pair<int, int>(56, 34));
  EXPECT_TRUE(map1[56] == 34);
  EXPECT_TRUE(map1.size() == 4);

  map1.insert(Pair<int, int>(56, 13));
  EXPECT_TRUE(map1[56] == 13);
  EXPECT_TRUE(map1.size() == 4);

  map1.remove(1);
  EXPECT_TRUE(map1.contains(1) == false);
  EXPECT_TRUE(map1.contains(8) == true);
  EXPECT_TRUE(map1.contains(13) == true);
  EXPECT_TRUE(map1.contains(56) == true);
  EXPECT_TRUE(map1.size() == 3);

  map1.remove(13);
  EXPECT_TRUE(map1.contains(13) == false);
  EXPECT_TRUE(map1.contains(8) == true);
  EXPECT_TRUE(map1.contains(56) == true);
  EXPECT_TRUE(map1.size() == 2);

  map1.remove(8);
  EXPECT_TRUE(map1.contains(8) == false);
  EXPECT_TRUE(map1.contains(56) == true);
  EXPECT_TRUE(map1.size() == 1);

  auto it = map1.remove(56);
  EXPECT_TRUE(map1.contains(56) == false);
  EXPECT_TRUE(map1.size() == 0);
  EXPECT_TRUE(it == map1.end());

  valid = true;

  for(int i = 0; i < 1000000; i++) {
    if(map2[i] != i * 100) {
      valid = false;
    }

    map2.remove(i);
  }

  for(int i = 0; i < 1000000; i++) {
    if(map2.contains(i)) {
      valid = false;
    }
  }

  EXPECT_TRUE(valid == true);
  EXPECT_TRUE(map2.size() == 0);

  Map<int, int> map3(memoryHandler);

  for (int i = 0; i < 1000000; i++) {
    map3.insert(Pair<int, int>(i, i * 10));
    map3.remove(i);
  }

  map3.insert(Pair<int, int>(1, 10));
  map3.insert(Pair<int, int>(2, 20));
  map3.insert(Pair<int, int>(3, 30));
  EXPECT_TRUE(map3.size() == 3);

  it = map3.begin();
  it = map3.remove(it);
  EXPECT_TRUE(map3.size() == 2);

  it = map3.remove(it);
  EXPECT_TRUE(map3.size() == 1);

  it = map3.remove(it);
  EXPECT_TRUE(map3.size() == 0);

  map3.insert(Pair<int, int>(56, 32));
  map3.insert(Pair<int, int>(23, 89));

  for (it = map3.begin(); it != map3.end();) {
    it = map3.remove(it);
  }

  EXPECT_TRUE(map3.size() == 0);

  Map<int, int> map4(memoryHandler);
  map4.insert(Pair<int, int>(2, 20));
  map4.insert(Pair<int, int>(4, 40));
  map4.insert(Pair<int, int>(6, 60));
  map4.clear();
  EXPECT_TRUE(map4.size() == 0);

  map4.insert(Pair<int, int>(2, 20));
  EXPECT_TRUE(map4.size() == 1);
  EXPECT_TRUE(map4[2] == 20);

  map4.clear();
  EXPECT_TRUE(map4.size() == 0);

  Map<int, int> map5(memoryHandler);
  map5.clear();
  EXPECT_TRUE(map5.size() == 0);

  Map<TestKey, int> map6(memoryHandler);

  for (int i = 0; i < 1000; i++) {
    map6.insert(Pair<TestKey, int>(TestKey(i), i));
  }

  valid = true;

  for (int i=0; i < 1000; i++) {
    if (map6[TestKey(i)] != i) {
      valid = false;
    }
  }

  EXPECT_TRUE(map6.size() == 1000);
  EXPECT_TRUE(valid == true);

  for (int i=0; i < 1000; i++) {
    if (map6[TestKey(i)] != i) {
      valid = false;
    }

    map6.remove(TestKey(i));
  }

  EXPECT_TRUE(map6.size() == 0);
}

TEST(Map, ContainsKey) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  EXPECT_TRUE(!map1.contains(2) == true);
  EXPECT_TRUE(!map1.contains(4) == true);
  EXPECT_TRUE(!map1.contains(6) == true);

  map1.insert(Pair<int, int>(2, 20));
  map1.insert(Pair<int, int>(4, 40));
  map1.insert(Pair<int, int>(6, 60));
  EXPECT_TRUE(map1.contains(2) == true);
  EXPECT_TRUE(map1.contains(4) == true);
  EXPECT_TRUE(map1.contains(6) == true);

  map1.remove(4);
  EXPECT_TRUE(map1.contains(4) == false);
  EXPECT_TRUE(map1.contains(2) == true);
  EXPECT_TRUE(map1.contains(6) == true);

  map1.clear();
  EXPECT_TRUE(map1.contains(2) == false);
  EXPECT_TRUE(map1.contains(6) == false);
}

TEST(Map, Indexing) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  map1.insert(Pair<int, int>(2, 20));
  map1.insert(Pair<int, int>(4, 40));
  map1.insert(Pair<int, int>(6, 60));
  EXPECT_TRUE(map1[2] == 20);
  EXPECT_TRUE(map1[4] == 40);
  EXPECT_TRUE(map1[6] == 60);

  map1[2] = 10;
  map1[4] = 20;
  map1[6] = 30;
  EXPECT_TRUE(map1[2] == 10);
  EXPECT_TRUE(map1[4] == 20);
  EXPECT_TRUE(map1[6] == 30);
}

TEST(Map, Find) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  map1.insert(Pair<int, int>(2, 20));
  map1.insert(Pair<int, int>(4, 40));
  map1.insert(Pair<int, int>(6, 60));
  EXPECT_TRUE(map1.find(2)->second == 20);
  EXPECT_TRUE(map1.find(4)->second == 40);
  EXPECT_TRUE(map1.find(6)->second == 60);
  EXPECT_TRUE(map1.find(45) == map1.end());

  map1[2] = 10;
  map1[4] = 20;
  map1[6] = 30;
  EXPECT_TRUE(map1.find(2)->second == 10);
  EXPECT_TRUE(map1.find(4)->second == 20);
  EXPECT_TRUE(map1.find(6)->second == 30);
}

TEST(Map, Equality) {
  VanillaMemoryHandler memoryHandler;
  Map<std::string, int> map1(memoryHandler);
  Map<std::string, int> map2(memoryHandler);
  EXPECT_TRUE(map1 == map2);

  map1.insert(Pair<std::string, int>("a", 1));
  map1.insert(Pair<std::string, int>("b", 2));
  map1.insert(Pair<std::string, int>("c", 3));
  map2.insert(Pair<std::string, int>("a", 1));
  map2.insert(Pair<std::string, int>("b", 2));
  map2.insert(Pair<std::string, int>("c", 4));
  EXPECT_TRUE(map1 == map1);
  EXPECT_TRUE(map2 == map2);
  EXPECT_TRUE(map1 != map2);

  map2["c"] = 3;
  EXPECT_TRUE(map1 == map2);

  Map<std::string, int> map3(memoryHandler);
  map3.insert(Pair<std::string, int>("a", 1));
  EXPECT_TRUE(map1 != map3);
  EXPECT_TRUE(map2 != map3);
}

TEST(Map, Assignment) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  map1.insert(Pair<int, int>(1, 3));
  map1.insert(Pair<int, int>(2, 6));
  map1.insert(Pair<int, int>(10, 30));
  Map<int, int> map2(memoryHandler);
  map2 = map1;
  EXPECT_TRUE(map2.size() == map1.size());
  EXPECT_TRUE(map1 == map2);
  EXPECT_TRUE(map2[1] == 3);
  EXPECT_TRUE(map2[2] == 6);
  EXPECT_TRUE(map2[10] == 30);

  Map<int, int> map3(memoryHandler);
  map3 = map1;
  EXPECT_TRUE(map3.size() == map1.size());
  EXPECT_TRUE(map3 == map1);
  EXPECT_TRUE(map3[1] == 3);
  EXPECT_TRUE(map3[2] == 6);
  EXPECT_TRUE(map3[10] == 30);

  Map<int, int> map4(memoryHandler);
  map3 = map4;
  EXPECT_TRUE(map3.size() == 0);
  EXPECT_TRUE(map3 == map4);

  Map<int, int> map5(memoryHandler);
  map5.insert(Pair<int, int>(7, 8));
  map5.insert(Pair<int, int>(19, 70));
  map1 = map5;
  EXPECT_TRUE(map5.size() == map1.size());
  EXPECT_TRUE(map5 == map1);
  EXPECT_TRUE(map1[7] == 8);
  EXPECT_TRUE(map1[19] == 70);
}

TEST(Map, Iterator) {
  VanillaMemoryHandler memoryHandler;
  Map<int, int> map1(memoryHandler);
  EXPECT_TRUE(map1.begin() == map1.end());

  map1.insert(Pair<int, int>(1, 5));
  map1.insert(Pair<int, int>(2, 6));
  map1.insert(Pair<int, int>(3, 8));
  map1.insert(Pair<int, int>(4, -1));
  Map<int, int>::Iterator itBegin = map1.begin();
  Map<int, int>::Iterator it = map1.begin();
  EXPECT_TRUE(itBegin == it);

  size_t size = 0;

  for (it = map1.begin(); it != map1.end(); ++it) {
      EXPECT_TRUE(map1.contains(it->first) == true);
      size++;
  }

  EXPECT_TRUE(map1.size() == size);
}