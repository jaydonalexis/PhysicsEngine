#include "UnitTests.h"

#include <physics/collections/List.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>

using namespace physics;

TEST(List, Constructor) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  EXPECT_TRUE(list1.size() == 0);
  EXPECT_TRUE(list1.empty() == true);
  EXPECT_TRUE(list1.getHead() == nullptr);
  EXPECT_TRUE(list1.getTail() == nullptr);
  
  List<int> list2(memoryHandler);
  list2.addFront(1);
  list2.addFront(2);
  list2.addBack(4);
  list2.addBack(3);
  EXPECT_TRUE(list2.empty() == false);
  EXPECT_TRUE(list2.size() == 4);
  EXPECT_TRUE(list2.getHead()->data == 2);
  EXPECT_TRUE(list2.getTail()->data == 3);
  
  List<int> list3(list2);
  EXPECT_TRUE(list3.size() == list2.size());
  EXPECT_TRUE(list3.empty() == false);

  List<int>::ListNode* node1 = list2.getHead();
  List<int>::ListNode* node2 = list3.getHead();

  for(uint32 i = 0; i < 4; i++) {
    EXPECT_TRUE(node1->data == node2->data);
    node1 = node1->next;
    node2 = node2->next;
  }
}

TEST(List, Modifiers) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  list1.addFront(4);
  EXPECT_TRUE(list1.size() == 1);
  EXPECT_TRUE(list1.getHead()->data == 4);
  EXPECT_TRUE(list1.getHead()->prev == nullptr);

  list1.addFront(9);
  EXPECT_TRUE(list1.size() == 2);
  EXPECT_TRUE(list1.getHead()->data == 9);
  EXPECT_TRUE(list1.getHead()->next->data == 4);
  EXPECT_TRUE(list1.getHead()->prev == nullptr);
  EXPECT_TRUE(list1.getTail()->data == 4);

  list1.addBack(5);
  EXPECT_TRUE(list1.size() == 3);
  EXPECT_TRUE(list1.getHead()->data == 9);
  EXPECT_TRUE(list1.getTail()->prev->data == 4);
  EXPECT_TRUE(list1.getTail()->data == 5);
  EXPECT_TRUE(list1.getTail()->next == nullptr);

  List<int> list2(memoryHandler);
  const int arraySize = 15;
  int arrayTest[arraySize] = {3, 145, -182, 34, 12, 95, -1834, 4143, -111, -111, 4343, 234, 22983, -3432, 753};
  
  for(int i = 0; i < arraySize; i++) {
      list2.addBack(arrayTest[i]);
  }

  EXPECT_TRUE(list2.size() == arraySize);
  EXPECT_TRUE(list2.getHead()->data == arrayTest[0]);
  EXPECT_TRUE(list2.getTail()->data == arrayTest[arraySize - 1]);

  List<int>::ListNode* head = list2.getHead();
  List<int>::ListNode* tail = list2.getTail();

  for(int i = 0; i < arraySize; i++) {
    EXPECT_TRUE(head->data == arrayTest[i]);
    head = head->next;
  }

  for(int i = arraySize - 1; i >= 0; i--) {
    EXPECT_TRUE(tail->data == arrayTest[i]);
    tail = tail->prev;
  }

  List<int> list3(memoryHandler);
  list3.addFront(1);
  list3.addFront(2);
  list3.addFront(3);
  list3.addFront(4);

  list3.removeFront();
  EXPECT_TRUE(list3.size() == 3);
  EXPECT_TRUE(list3.getHead()->data == 3);
  EXPECT_TRUE(list3.getHead()->prev == nullptr);
  EXPECT_TRUE(list3.getHead()->next->data == 2);

  list3.removeBack();
  EXPECT_TRUE(list3.size() == 2);
  EXPECT_TRUE(list3.getTail()->data == 2);
  EXPECT_TRUE(list3.getTail()->next == nullptr);
  EXPECT_TRUE(list3.getTail()->prev->data == 3);

  list3.removeFront();
  EXPECT_TRUE(list3.size() == 1);
  EXPECT_TRUE(list3.getHead()->data == 2);

  list3.removeFront();
  EXPECT_TRUE(list3.size() == 0);
  EXPECT_TRUE(list3.empty() == true);

  list3.removeBack();
  EXPECT_TRUE(list3.size() == 0);
  EXPECT_TRUE(list3.empty());

  list3.removeFront();
  EXPECT_TRUE(list3.size() == 0);
  EXPECT_TRUE(list3.empty());

  List<int> list4(memoryHandler);
  list4.addFront(1);
  list4.addFront(2);
  list4.addFront(3);
  list4.addFront(4);
  EXPECT_TRUE(list4.size() == 4);

  list4.clear();
  EXPECT_TRUE(list4.size() == 0);
  EXPECT_TRUE(list4.empty());
  EXPECT_TRUE(list4.getHead() == nullptr);
  EXPECT_TRUE(list4.getTail() == nullptr);

  List<std::string> list5(memoryHandler);
  list5.addBack("test1");
  list5.addBack("test2");
  list5.addBack("test3");
  list5.clear();
  EXPECT_TRUE(list5.size() == 0);

  list5.addBack("new");
  EXPECT_TRUE(list5.size() == 1);
  EXPECT_TRUE(list5.getHead()->data == "new");
  EXPECT_TRUE(list5.getTail()->data == "new");
}

TEST(List, Iterator) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  EXPECT_TRUE(list1.begin() == list1.end());

  list1.addFront(5);
  list1.addBack(6);
  list1.addBack(8);
  list1.addBack(-1);
  List<int>::Iterator begin = list1.begin();
  List<int>::Iterator end = list1.end();
  List<int>::Iterator it = list1.begin();
  EXPECT_TRUE(begin != end);
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

  it = list1.end();
  EXPECT_TRUE(it == end);

  it--;
  EXPECT_TRUE(*it == -1);

  it++;
  EXPECT_TRUE(it == end);

  List<int> list2(memoryHandler);

  for(it = list1.begin(); it != list1.end(); ++it) {
    list2.addBack(*it);
  }

  EXPECT_TRUE(list1.size() == list2.size());

  auto it1 = list1.begin();
  auto it2 = list2.begin();

  while(it1 != list1.end() && it2 != list2.end()) {
    EXPECT_TRUE(*it1 == *it2);
    ++it1;
    ++it2;
  }

  it = begin;
  *it = 19;
  EXPECT_TRUE(*it == 19);
  EXPECT_TRUE(list1.getHead()->data == 19);
}

TEST(List, Assignment) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  list1.addFront(1);
  list1.addFront(2);
  list1.addFront(3);

  List<int> list2(memoryHandler);
  list2.addBack(5);
  list2.addBack(6);

  List<int> list3(memoryHandler);
  List<int> list4(memoryHandler);
  list4.addBack(1);
  list4.addBack(2);

  List<int> list5(memoryHandler);
  list5.addFront(1);
  list5.addFront(2);
  list5.addFront(3);

  list3 = list2;
  EXPECT_TRUE(list2.size() == list3.size());

  auto it1 = list2.begin();
  auto it2 = list3.begin();

  while(it1 != list2.end() && it2 != list3.end()) {
    EXPECT_TRUE(*it1 == *it2);
    ++it1;
    ++it2;
  }

  list4 = list1;
  EXPECT_TRUE(list4.size() == list1.size());

  it1 = list1.begin();
  it2 = list4.begin();

  while(it1 != list1.end() && it2 != list4.end()) {
    EXPECT_TRUE(*it1 == *it2);
    ++it1;
    ++it2;
  }

  list5 = list2;
  EXPECT_TRUE(list5.size() == list2.size());

  it1 = list2.begin();
  it2 = list5.begin();

  while(it1 != list2.end() && it2 != list5.end()) {
    EXPECT_TRUE(*it1 == *it2);
    ++it1;
    ++it2;
  }
}

TEST(List, Insert) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  list1.insert(list1.begin(), 1);
  list1.insert(list1.begin(), 2);
  EXPECT_TRUE(list1.getHead()->data == 2);
  EXPECT_TRUE(list1.getTail()->data == 1);

  List<int>::Iterator iter = list1.begin();
  iter++;
  list1.insert(iter, 3);
  EXPECT_TRUE(list1.getHead()->next->data == 3);
  EXPECT_TRUE(list1.getTail()->prev->data == 3);
}

TEST(List, Erase) {
  VanillaMemoryHandler memoryHandler;
  List<int> list1(memoryHandler);
  list1.insert(list1.begin(), 1);
  list1.insert(list1.begin(), 2);
  list1.insert(list1.end(), 3);
  EXPECT_TRUE(list1.getHead()->data == 2);
  EXPECT_TRUE(list1.getTail()->data == 3);

  List<int>::Iterator iter = list1.begin();
  iter++;

  list1.erase(iter);
  EXPECT_TRUE(list1.getTail()->data == 3);
  EXPECT_TRUE(list1.getHead()->data == 2);

  list1.erase(list1.begin());
  EXPECT_TRUE(list1.getHead()->data == 3);
  EXPECT_TRUE(list1.getTail()->data == 3);

  list1.erase(list1.begin());
  EXPECT_TRUE(list1.empty() == true);
  EXPECT_TRUE(list1.size() == 0);
  EXPECT_TRUE(list1.getHead() == nullptr);
  EXPECT_TRUE(list1.getTail() == nullptr);
}