#include "UnitTests.h"

#include <physics/collections/DynamicArray.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(DynamicArray, Constructors) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  EXPECT_TRUE(array1.size() == 0);
  EXPECT_TRUE(array1.capacity() == 0);

  DynamicArray<int> array2(memoryHandler, 100);
  EXPECT_TRUE(array2.size() == 0);
  EXPECT_TRUE(array2.capacity() == 100);

  DynamicArray<int> array3(memoryHandler);
  array3.add(1);
  array3.add(2);
  array3.add(3);
  EXPECT_TRUE(array3.capacity() == 4);
  EXPECT_TRUE(array3.size() == 3);

  DynamicArray<int> array4(array1);
  EXPECT_TRUE(array4.size() == 0);
  EXPECT_TRUE(array4.capacity() == 0);

  DynamicArray<int> array5(array3);
  EXPECT_TRUE(array5.capacity() == array3.capacity());
  EXPECT_TRUE(array5.size() == array3.size());

  for (size_t i = 0; i < array3.size(); i++) {
      EXPECT_TRUE(array5[i] == array3[i]);
  }

  DynamicArray<std::string> array6(memoryHandler, 20);
  EXPECT_TRUE(array6.capacity() == 20);

  for (int i = 0; i < 20; i++) {
      array6.add("test");
  }

  EXPECT_TRUE(array6.capacity() == 20);
  array6.add("test");
  EXPECT_TRUE(array6.capacity() == 40);
}

TEST(DynamicArray, Modifiers) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.add(4);
  EXPECT_TRUE(array1.size() == 1);
  EXPECT_TRUE(array1[0] == 4);
  
  array1.add(9);
  EXPECT_TRUE(array1.size() == 2);
  EXPECT_TRUE(array1[0] == 4);
  EXPECT_TRUE(array1[1] == 9);

  const int arraySize = 15;
  int arrayTest[arraySize] = {3, 145, -182, 34, 12, 95, -1834, 4143, -111, -111, 4343, 234, 22983, -3432, 753};
  DynamicArray<int> array2(memoryHandler);

  for (int i = 0; i < arraySize; i++) {
      array2.add(arrayTest[i]);
  }

  EXPECT_TRUE(array2.size() == arraySize);

  for (int i = 0; i < arraySize; i++) {
      EXPECT_TRUE(array2[i] == arrayTest[i]);
  }

  DynamicArray<int> array3(memoryHandler);
  array3.add(1);
  array3.add(2);
  array3.add(3);
  array3.add(4);

  auto it = array3.erase(3);
  EXPECT_TRUE(array3.size() == 3);
  EXPECT_TRUE(array3.capacity() == 4);
  EXPECT_TRUE(it == array3.end());
  EXPECT_TRUE(array3[0] == 1);
  EXPECT_TRUE(array3[1] == 2);
  EXPECT_TRUE(array3[2] == 3);

  it = array3.erase(1);
  EXPECT_TRUE(array3.size() == 2);
  EXPECT_TRUE(array3.capacity() == 4);
  EXPECT_TRUE(array3[0] == 1);
  EXPECT_TRUE(array3[1] == 3);
  EXPECT_TRUE(*it == 3);

  array3.erase(0);
  EXPECT_TRUE(array3.size() == 1);
  EXPECT_TRUE(array3.capacity() == 4);
  EXPECT_TRUE(array3[0] == 3);

  it = array3.erase(0);
  EXPECT_TRUE(array3.size() == 0);
  EXPECT_TRUE(array3.capacity() == 4);
  EXPECT_TRUE(it == array3.end());

  array3.add(1);
  array3.add(2);
  array3.add(3);
  it = array3.begin();
  array3.remove(it);
  EXPECT_TRUE(array3.size() == 2);
  EXPECT_TRUE(array3[0] == 2);
  EXPECT_TRUE(array3[1] == 3);

  it = array3.find(3);
  array3.remove(it);
  EXPECT_TRUE(array3.size() == 1);
  EXPECT_TRUE(array3[0] == 2);

  array3.add(5);
  array3.add(6);
  array3.add(7);
  it = array3.remove(7);
  EXPECT_TRUE(it == array3.end());
  EXPECT_TRUE(array3.size() == 3);

  it = array3.remove(5);
  EXPECT_TRUE((*it) == 6);

  DynamicArray<int> array4(memoryHandler);
  array4.add(1);
  array4.add(2);
  array4.add(3);

  DynamicArray<int> array5(memoryHandler);
  array5.add(4);
  array5.add(5);

  DynamicArray<int> array6(memoryHandler);
  array6.add(array5);
  EXPECT_TRUE(array6.size() == array5.size());
  EXPECT_TRUE(array6[0] == 4);
  EXPECT_TRUE(array6[1] == 5);

  array4.add(array5);
  EXPECT_TRUE(array4.size() == 3 + array5.size());
  EXPECT_TRUE(array4[0] == 1);
  EXPECT_TRUE(array4[1] == 2);
  EXPECT_TRUE(array4[2] == 3);
  EXPECT_TRUE(array4[3] == 4);
  EXPECT_TRUE(array4[4] == 5);

  DynamicArray<std::string> array7(memoryHandler);
  array7.add("test1");
  array7.add("test2");
  array7.add("test3");
  array7.clear();
  EXPECT_TRUE(array7.size() == 0);

  array7.add("new");
  EXPECT_TRUE(array7.size() == 1);
  EXPECT_TRUE(array7[0] == "new");
}

TEST(DynamicArray, Assignment) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.add(1);
  array1.add(2);
  array1.add(3);

  DynamicArray<int> array2(memoryHandler);
  array2.add(5);
  array2.add(6);

  DynamicArray<int> array3(memoryHandler);
  DynamicArray<int> array4(memoryHandler);
  array4.add(1);
  array4.add(2);

  DynamicArray<int> array5(memoryHandler);
  array5.add(1);
  array5.add(2);
  array5.add(3);

  array3 = array2;
  EXPECT_TRUE(array2.size() == array3.size());
  EXPECT_TRUE(array2[0] == array3[0]);
  EXPECT_TRUE(array2[1] == array3[1]);

  array4 = array1;
  EXPECT_TRUE(array4.size() == array1.size());
  EXPECT_TRUE(array4[0] == array1[0]);
  EXPECT_TRUE(array4[1] == array1[1]);
  EXPECT_TRUE(array4[2] == array1[2]);

  array5 = array2;
  EXPECT_TRUE(array5.size() == array2.size());
  EXPECT_TRUE(array5[0] == array2[0]);
  EXPECT_TRUE(array5[1] == array2[1]);
}

TEST(DynamicArray, Indexing) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.add(1);
  array1.add(2);
  array1.add(3);
  EXPECT_TRUE(array1[0] == 1);
  EXPECT_TRUE(array1[1] == 2);
  EXPECT_TRUE(array1[2] == 3);

  array1[0] = 6;
  array1[1] = 7;
  array1[2] = 8;
  EXPECT_TRUE(array1[0] == 6);
  EXPECT_TRUE(array1[1] == 7);
  EXPECT_TRUE(array1[2] == 8);

  const int a = array1[0];
  const int b = array1[1];
  EXPECT_TRUE(a == 6);
  EXPECT_TRUE(b == 7);

  array1[0]++;
  array1[1]++;
  EXPECT_TRUE(array1[0] == 7);
  EXPECT_TRUE(array1[1] == 8);
}

TEST(DynamicArray, Find) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.add(1);
  array1.add(2);
  array1.add(3);
  array1.add(4);
  array1.add(5);
  EXPECT_TRUE(array1.find(1) == array1.begin());
  EXPECT_TRUE(*(array1.find(2)) == 2);
  EXPECT_TRUE(*(array1.find(5)) == 5);
}

TEST(DynamicArray, Equality) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.add(1);
  array1.add(2);
  array1.add(3);

  DynamicArray<int> array2(memoryHandler);
  array2.add(1);
  array2.add(2);

  DynamicArray<int> array3(memoryHandler);
  array3.add(1);
  array3.add(2);
  array3.add(3);

  DynamicArray<int> array4(memoryHandler);
  array4.add(1);
  array4.add(5);
  array4.add(3);

  EXPECT_TRUE(array1 == array1);
  EXPECT_TRUE(array1 != array2);
  EXPECT_TRUE(array1 == array3);
  EXPECT_TRUE(array1 != array4);
  EXPECT_TRUE(array2 != array4);
}

TEST(DynamicArray, Reserve) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  array1.reserve(10);
  EXPECT_TRUE(array1.size() == 0);
  EXPECT_TRUE(array1.capacity() == 10);

  array1.add(1);
  array1.add(2);
  EXPECT_TRUE(array1.capacity() == 10);
  EXPECT_TRUE(array1.size() == 2);
  EXPECT_TRUE(array1[0] == 1);
  EXPECT_TRUE(array1[1] == 2);

  array1.reserve(1);
  EXPECT_TRUE(array1.capacity() == 10);

  array1.reserve(100);
  EXPECT_TRUE(array1.capacity() == 100);
  EXPECT_TRUE(array1[0] == 1);
  EXPECT_TRUE(array1[1] == 2);
}

TEST(DynamicArray, Iterator) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> array1(memoryHandler);
  EXPECT_TRUE(array1.begin() == array1.end());

  array1.add(5);
  array1.add(6);
  array1.add(8);
  array1.add(-1);
  DynamicArray<int>::Iterator begin = array1.begin();
  DynamicArray<int>::Iterator end = array1.end();
  DynamicArray<int>::Iterator it = array1.begin();

  EXPECT_TRUE(begin < end);
  EXPECT_TRUE(begin <= end);
  EXPECT_TRUE(end > begin);
  EXPECT_TRUE(end >= begin);
  EXPECT_TRUE(begin == it);
  EXPECT_TRUE(*it == 5);
  EXPECT_TRUE(*(it++) == 5);
  EXPECT_TRUE(*it == 6);
  EXPECT_TRUE(*(it--) == 6);
  EXPECT_TRUE(*it == 5);
  EXPECT_TRUE(*(++it) == 6);
  EXPECT_TRUE(*it == 6);
  EXPECT_TRUE(*(--it) == 5);
  EXPECT_TRUE(*it == 5);
  EXPECT_TRUE(it == begin);

  it = array1.end();
  EXPECT_TRUE(it == end);

  it--;
  EXPECT_TRUE(*it == -1);

  it++;
  EXPECT_TRUE(it == end);

  DynamicArray<int> array2(memoryHandler);

  for (auto it = array1.begin(); it != array1.end(); ++it) {
      array2.add(*it);
  }

  EXPECT_TRUE(array1 == array2);

  it = begin;
  EXPECT_TRUE(*(it + 2) == 8);

  it += 2;
  EXPECT_TRUE(*it == 8);
  EXPECT_TRUE(*(it - 2) == 5);

  it -= 2;
  EXPECT_TRUE(*it == 5);
  EXPECT_TRUE((end - begin) == 4);

  it = begin;
  *it = 19;
  EXPECT_TRUE(*it == 19);
}