#ifndef PHYSICS_DYNAMIC_TREE_H
#define PHYSICS_DYNAMIC_TREE_H

#include <physics/Configuration.h>
#include <physics/collision/AABB.h>
#include <physics/collections/HashSet.h>
#include <physics/collections/DynamicArray.h>
#include <physics/collections/Pair.h>

#define NULL_NODE -1
#define FREE_NODE_HEIGHT -1
#define LEAF_HEIGHT 0

namespace physics {

/* Forward declarations */
class BroadPhase;
class OverlapCallback;
class AABB;
class MemoryHandler;

/* Node for the dynamic tree */
struct Node {
  
  public:
    /* -- Attributes -- */

    /* Enlarged AABB */
    AABB aabb;

    /* User data */
    void* data;

    /* Height of the current node in the tree */
    int32 height;

    /* A node can either be present in the tree or it can be in the list of free nodes */
    union {
      /* Parent in tree */
      int32 parent;

      /* Next in free nodes list */
      int32 next;
    };

    /* Left child in the tree */
    int32 leftChild;
    
    /* Right child in the tree */
    int32 rightChild;

    /* -- Methods -- */

    /* Constructor */
    Node() : next(NULL_NODE), height(FREE_NODE_HEIGHT) {}

    /* Query whether the current node is a leaf node */
    bool isLeaf() const {
      return height == 0;
    }
};

class DynamicTree {

  private:
    /* -- Attributes -- */

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* Array of tree nodes */
    Node* mNodes;

    /* Root node */
    int32 mRoot;

    /* Head of the list of free nodes */
    int32 mFree;

    /* Number of allocated nodes in tree */
    int32 mNumAllocatedNodes;

    /* Number of active nodes in tree */
    int mNumNodes;

    /* Enlarged AABB inflation */
    float mFatAABBInflation;

  private:
    /* -- Methods -- */

    /* Initialization function */
    void initialize();

    /* Compute the height of the given node in the tree */
    int32 getNodeHeight(int32 node);

    /* Allocate a node */
    int32 createNode();

    /* Extract a node */
    void extractNode(int32 node);

    /* Insert a node as a leaf in the tree */
    void insertLeaf(int32 node);

    /* Remove a leaf node from the tree */
    void removeLeaf(int32 node);

    /* Balance a section of the tree with the given node as the pivot */
    int32 balance(int32 node);

    /* Insert an object into the tree given it's AABB */
    int32 insertObject(const AABB& aabb);

  public:
    /* -- Methods -- */

    /* Constructor */
    DynamicTree(MemoryHandler& memoryHandler, float fatAABBInflation = 0.0f);

    /* Destructor */
    ~DynamicTree();

    /* Compute the height of the tree */
    int32 height();

    /* Get the root AABB */
    AABB getRootAABB() const;

    /* Get a node's enlarged AABB */
    const AABB& getFatAABB(int32 node) const;

    /* Get data associated with the given node */
    void* getNodeData(int32 node) const;

    /* Add an object into the tree */
    int32 add(const AABB& aabb, void* data);

    /* Remove an object from the tree */
    void remove(int32 node);

    /* Update object when it has moved */
    bool update(int32 node, const AABB& aabb, bool force = false);

    /* Get all of the shapes that are overlapping with the provided test shapes */
    void getShapeShapeOverlaps(const DynamicArray<int32>& testNodes, uint32 begin, uint32 end, DynamicArray<Pair<int32, int32>>& overlappingNodes) const;

    /* Get all of the shapes that are overlapping with the provided AABB */
    void getShapeAABBOverlap(const AABB& aabb, DynamicArray<int32>& overlappingNodes) const;

    /* Clear the tree */
    void clear();
};

}

#endif