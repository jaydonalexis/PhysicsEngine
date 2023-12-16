#include "UnitTests.h"

#include <physics/collision/DynamicTree.h>
#include <physics/collections/DynamicArray.h>
#include <physics/memory/Vanilla.h>

#include <sstream>
#include <type_traits>
#include <algorithm>
#include <cstddef>
#include <vector>

using namespace physics;

TEST(DynamicTree, BasicFunctionality) {
  VanillaMemoryHandler memoryHandler;
  DynamicTree tree(memoryHandler);
  int data1 = 56;
  int data2 = 23;
  int data3 = 13;
  int data4 = 7;
  AABB aabb1(Vector2(-6.0f, 4.0f), Vector2(4.0f, 8.0f));
  int identifier1 = tree.add(aabb1, &data1);
  AABB aabb2(Vector2(5.0f, 2.0f), Vector2(10.0f, 7.0f));
  int identifier2 = tree.add(aabb2, &data2);
  AABB aabb3(Vector2(-5.0f, 1.0f), Vector2(-2.0f, 3.0f));
  int identifier3 = tree.add(aabb3, &data3);
  AABB aabb4(Vector2(0.0f, -4.0f), Vector2(3.0f, -2.0f));
  int identifier4 = tree.add(aabb4, &data4);

  AABB root = tree.getRootAABB();
  EXPECT_EQ(root.getlowerBound(), Vector2(-6.0f, -4.0f));
  EXPECT_EQ(root.getUpperBound(), Vector2(10.0f, 8.0f));

  EXPECT_TRUE(*(int*)(tree.getNodeData(identifier1)) == data1);
  EXPECT_TRUE(*(int*)(tree.getNodeData(identifier2)) == data2);
  EXPECT_TRUE(*(int*)(tree.getNodeData(identifier3)) == data3);
  EXPECT_TRUE(*(int*)(tree.getNodeData(identifier4)) == data4);
}

TEST(DynamicTree, Overlap) {
  VanillaMemoryHandler memoryHandler;
  DynamicArray<int> overlapNodes(memoryHandler);
  DynamicTree tree(memoryHandler);
  int data1 = 56;
  int data2 = 23;
  int data3 = 13;
  int data4 = 7;
  AABB aabb1(Vector2(-6.0f, 4.0f), Vector2(4.0f, 8.0f));
  int identifier1 = tree.add(aabb1, &data1);
  AABB aabb2(Vector2(5.0f, 2.0f), Vector2(10.0f, 7.0f));
  int identifier2 = tree.add(aabb2, &data2);
  AABB aabb3(Vector2(-5.0f, 1.0f), Vector2(-2.0f, 3.0f));
  int identifier3 = tree.add(aabb3, &data3);
  AABB aabb4(Vector2(0.0f, -4.0f), Vector2(3.0f, -2.0f));
  int identifier4 = tree.add(aabb4, &data4);

  /* No overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(12.0f, -6.0f), Vector2(24.0f, 10.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* All overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-8.0f, -6.0f), Vector2(12.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 1 & 3 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, 0.0f), Vector2(2.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 3 & 4 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, -3.0f), Vector2(2.0f, 2.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap object 2 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(8.0f, 2.0f), Vector2(10.0f, 7.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Update with no reinsertion */
  tree.update(identifier1, aabb1);
  tree.update(identifier2, aabb2);
  tree.update(identifier3, aabb3);
  tree.update(identifier4, aabb4);

  /* No overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(12.0f, -6.0f), Vector2(24.0f, 10.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* All overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-8.0f, -6.0f), Vector2(12.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 1 & 3 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, 0.0f), Vector2(2.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 3 & 4 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, -3.0f), Vector2(2.0f, 2.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap object 2 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(8.0f, 2.0f), Vector2(10.0f, 7.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Update with reinsertion */
  tree.update(identifier1, aabb1, true);
  tree.update(identifier2, aabb2, true);
  tree.update(identifier3, aabb3, true);
  tree.update(identifier4, aabb4, true);

  /* No overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(12.0f, -6.0f), Vector2(24.0f, 10.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* All overlap */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-8.0f, -6.0f), Vector2(12.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 1 & 3 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, 0.0f), Vector2(2.0f, 10.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 3 & 4 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(-4.0f, -3.0f), Vector2(2.0f, 2.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap object 2 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(8.0f, 2.0f), Vector2(10.0f, 7.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Move objects 2 & 3 */
  AABB aabb2New(Vector2(-7.0f, 10.0f), Vector2(1.0f, 13.0f));
  tree.update(identifier2, aabb2New);
  AABB aabb3New(Vector2(7.0f, -6.0f), Vector2(9.0f, 1.0f));
  tree.update(identifier3, aabb3New);

  /* Overlap object 3 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(8.0f, 0.0f), Vector2(10.0f, 2.0f)), overlapNodes);
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());

  /* Overlap objects 1 & 2 */
  overlapNodes.clear();
  tree.getShapeAABBOverlap(AABB(Vector2(0.0f, 4.0f), Vector2(4.0f, 13.0f)), overlapNodes);
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier1) != overlapNodes.end());
  EXPECT_TRUE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier2) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier3) != overlapNodes.end());
  EXPECT_FALSE(std::find(overlapNodes.begin(), overlapNodes.end(), identifier4) != overlapNodes.end());
}