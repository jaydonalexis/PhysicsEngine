#include <physics/collision/DynamicTree.h>
#include <physics/collections/Stack.h>
#include <physics/common/Factory.h>

using namespace physics;

/* Constructor */
DynamicTree::DynamicTree(MemoryHandler& memoryHandler, float fatAABBInflation) : mMemoryHandler(memoryHandler), mFatAABBInflation(fatAABBInflation) {
  initialize();
}

/* Destructor */
DynamicTree::~DynamicTree() {
  /* Release memory for nodes */
  mMemoryHandler.free(mNodes, static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node));
}

/* Initialization function */
void DynamicTree::initialize() {
  mRoot = NULL_NODE;
  mNumNodes = 0;
  mNumAllocatedNodes = 8;

  /* Allocate, construct and initialize nodes */
  mNodes = static_cast<Node*>(mMemoryHandler.allocate(static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node)));
  assert(mNodes);

  for(int32 i = 0; i < mNumAllocatedNodes; i++) {
    new (mNodes + i) Node();
  }

  for(int32 i = 0; i < mNumAllocatedNodes; i++) {
    if(i == mNumAllocatedNodes - 1) {
      mNodes[i].next = NULL_NODE;
      mNodes[i].height = FREE_NODE_HEIGHT;
      continue;
    }

    mNodes[i].next = i + 1;
    mNodes[i].height = FREE_NODE_HEIGHT;
  }

  mFree = 0;
}

/* Compute the height of a given node in the tree */
int32 DynamicTree::getNodeHeight(int32 node) {
  assert(node >= 0 && node < mNumAllocatedNodes);
  Node* root = mNodes + node;

  if(root->isLeaf()) {
    return 0;
  }

  /* Recursively obtain the height of the left and right sub-trees */
  int32 heightLeft = getNodeHeight(root->leftChild);
  int32 heightRight = getNodeHeight(root->rightChild);

  /* Return the height of the node using the maximum height of the left and right sub-trees */
  return 1 + std::max(heightLeft, heightRight);
}

/* Allocate a node */
int32 DynamicTree::createNode() {
  if(mFree == NULL_NODE) {
    assert(mNumNodes == mNumAllocatedNodes);
    int32 numAllocatedNodesPrev = mNumAllocatedNodes;
    mNumAllocatedNodes *= 2;
    Node* nodesPrev = mNodes;
    mNodes = static_cast<Node*>(mMemoryHandler.allocate(static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node)));
    assert(mNodes);
    std::uninitialized_copy(nodesPrev, nodesPrev + mNumNodes, mNodes);
    mMemoryHandler.free(nodesPrev, static_cast<size_t>(numAllocatedNodesPrev) * sizeof(Node));

    /* Initialize newly allocated nodes */
    for(int32 i = mNumNodes; i < mNumAllocatedNodes - 1; i++) {
      new (mNodes + i) Node();
      
      if(i == mNumAllocatedNodes - 1) {
        mNodes[i].next = NULL_NODE;
        mNodes[i].height = FREE_NODE_HEIGHT;
        continue;
      }

      mNodes[i].next = i + 1;
      mNodes[i].height = FREE_NODE_HEIGHT;
    }

    mFree = mNumNodes;
  }

  /* Get the next free node in the array */
  int32 free = mFree;
  mFree = mNodes[free].next;
  mNodes[free].parent = NULL_NODE;
  mNodes[free].height = LEAF_HEIGHT;
  mNumNodes++;
  LOG("The dynamic tree currently contains " + std::to_string(mNumNodes) + " node(s)");
  return free;
}

/* Release a node (this does not mean deallocation) */
void DynamicTree::extractNode(int32 node) {
  assert(mNumNodes > 0);
  assert(node >= 0 && node< mNumAllocatedNodes);
  assert(mNodes[node].height >= 0);
  mNodes[node].next = mFree;
  mNodes[node].height = FREE_NODE_HEIGHT;
  mFree = node;
  mNumNodes--;
  LOG("The dynamic tree currently contains " + std::to_string(mNumNodes) + " node(s)");
}

/* Insert a node as a leaf in the tree */
void DynamicTree::insertLeaf(int32 node) {
  /* Tree is empty */
  if(mRoot == NULL_NODE) {
    mRoot = node;
    mNodes[mRoot].parent = NULL_NODE;
    return;
  }

  assert(mRoot != NULL_NODE);

  /* Find best sibling for node */
  AABB leafAABB = mNodes[node].aabb;
  int32 walk = mRoot;

  while(!mNodes[walk].isLeaf()) {
    int32 leftChild = mNodes[walk].leftChild;
    int32 rightChild = mNodes[walk].rightChild;

    /* Perimeter used over area as it is computationally cheaper */
    float area = mNodes[walk].aabb.getPerimeter();
    AABB combinedAABB;
    combinedAABB.combine(mNodes[walk].aabb, leafAABB);
    float combinedArea = combinedAABB.getPerimeter();

    /*
     * Cost for creating a new parent for current node and new leaf
     * The cost is defined as the sum over all of its children of the
     * probability of having to do an AABB intersection test between
     * the node to be inserted and each of the children which is the area
     * of the current node divided by the world area. The factor is two is
     * due to the fact that we have to test both children
     */
    float costSibling = 2.0f * combinedArea;

    /* 
     * It might be cheaper to push the node down to one of the children
     * That would increase the area by an amount equal to the combined
     * area of the inserted node and the current node minus the area of
     * the current node. Therefore, the inheritance cost is as follows
     */
    float costInheritance = 2.0f * (combinedArea - area);

    /* Cost of descending into left child */
    float costLeft;
    AABB leftAABB;
    leftAABB.combine(leafAABB, mNodes[leftChild].aabb);

    /* If the node is a leaf node, we replace it with a new node P and make C and N the children of P */
    if(mNodes[leftChild].isLeaf()) {
      /* Note that combined area is not doubled here due to the fact that this is a case where we visit a specific child of S */
      costLeft = leftAABB.getPerimeter() + costInheritance;
    }
    /* Otherwise, the cost is at least the following */
    else {
      costLeft = leftAABB.getPerimeter() - mNodes[leftChild].aabb.getPerimeter() + costInheritance;
    }

    /* Cost of descending into right child */
    float costRight;
    AABB rightAABB;
    rightAABB.combine(leafAABB, mNodes[rightChild].aabb);

    /* If the node is a leaf node, we replace it with a new node P and make C and N the children of P */
    if(mNodes[rightChild].isLeaf()) {
      /* Note that combined area is not doubled here due to the fact that this is a case where we visit a specific child of S */
      costRight = rightAABB.getPerimeter() + costInheritance;
    }
    /* Otherwise, the cost is at least the following */
    else {
      costRight = rightAABB.getPerimeter() - mNodes[rightChild].aabb.getPerimeter() + costInheritance;
    }

    /* It is not cheaper to push to one of the children */
    if(costSibling < costLeft && costSibling < costRight) {
      break;
    }

    if(costLeft < costRight) {
      walk = leftChild;
    }
    else {
      walk = rightChild;
    }
  }

  /* Create parent P for new node N and sibling node C */
  int32 sibling = walk;
  int32 parentPrev = mNodes[sibling].parent;
  int32 parent = createNode();
  mNodes[parent].parent = parentPrev;
  mNodes[parent].aabb.combine(mNodes[sibling].aabb, leafAABB);
  mNodes[parent].height = mNodes[sibling].height + 1;

  /* Sibling node was not the root node */
  if(parentPrev != NULL_NODE) {
    assert(!mNodes[parentPrev].isLeaf());

    if(mNodes[parentPrev].leftChild == sibling) {
      mNodes[parentPrev].leftChild = parent;
    }
    else {
      mNodes[parentPrev].rightChild = parent;
    }
  }
  /* Sibling node was the root node */
  else {
    mRoot = parent;
  }

  mNodes[parent].leftChild = sibling;
  mNodes[parent].rightChild = node;
  mNodes[sibling].parent = parent;
  mNodes[node].parent = parent;
  walk = mNodes[node].parent;

  assert(walk != NULL_NODE);

  /* Now that the node is inserted, we need to propogate corrective measures up the tree */
  while(walk != NULL_NODE) {
    /* Balance at the current node if not already */
    walk = balance(walk);
    assert(mNodes[node].isLeaf());
    assert(!mNodes[walk].isLeaf());
    int32 leftChild = mNodes[walk].leftChild;
    int32 rightChild = mNodes[walk].rightChild;
    assert(leftChild != NULL_NODE);
    assert(rightChild != NULL_NODE);
    /* Need to recalculate height as well as the AABB */
    mNodes[walk].height = 1 + std::max(mNodes[leftChild].height, mNodes[rightChild].height);
    assert(mNodes[walk].height > 0);
    mNodes[walk].aabb.combine(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
    walk = mNodes[walk].parent;
  }

  assert(mNodes[node].isLeaf());
}


/* Remove a leaf node from the tree */
void DynamicTree::removeLeaf(int32 node) {
  assert(node >= 0 && node < mNumAllocatedNodes);
  assert(mNodes[node].isLeaf());

  /* Root node case */
  if(mRoot == node) {
    mRoot = NULL_NODE;
    return;
  }

  int32 parent = mNodes[node].parent;
  int32 grandparent = mNodes[parent].parent;
  int32 sibling = mNodes[parent].leftChild == node ? mNodes[parent].rightChild : mNodes[parent].leftChild;

  /* Parent of the node to remove is not root */
  if(grandparent != NULL_NODE) {
    /* Destroy parent */
    if(mNodes[grandparent].leftChild == parent) {
      mNodes[grandparent].leftChild = sibling;
    }
    else {
      mNodes[grandparent].rightChild = sibling;
    }

    mNodes[sibling].parent = grandparent;
    extractNode(parent);
    int32 walk = grandparent;

    /* Now that the node is deleted, we need to propogate corrective measures up the tree */
    while(walk != NULL_NODE) {
      /* Balance at the current node if not already */
      walk = balance(walk);
      assert(!mNodes[walk].isLeaf());
      int32 leftChild = mNodes[walk].leftChild;
      int32 rightChild = mNodes[walk].rightChild;
      /* Need to recalculate height as well as the AABB */
      mNodes[walk].height = 1 + std::max(mNodes[leftChild].height, mNodes[rightChild].height);
      mNodes[walk].aabb.combine(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
      assert(mNodes[walk].height > 0);
      walk = mNodes[walk].parent;
    }
  }
  else {
    /* Parent of the node to remove is root */
    mRoot = sibling;
    mNodes[sibling].parent = NULL_NODE;
    extractNode(parent);
  }
}

/* Balance a section of the tree with the given node as the pivot */
int32 DynamicTree::balance(int32 node) {
  assert(node != NULL_NODE);
  Node* A = mNodes + node;
  
  if(A->isLeaf() || A->height < MINIMUM_BALANCE_DEPTH) {
    return node;
  }

  int32 iB = A->leftChild;
  int32 iC = A->rightChild;
  Node* B = mNodes + iB;
  Node* C = mNodes + iC;
  int32 balance = C->height - B->height;

  /* Bubble C up if higher than node B by 2 */
  if(balance > 1) {
    assert(!C->isLeaf());
    int32 iF = C->leftChild;
    int32 iG = C->rightChild;
    Node* F = mNodes + iF;
    Node* G = mNodes + iG;

    /* Swap node A and node C */
    C->leftChild = node;
    C->parent = A->parent;
    A->parent = iC;

    /* Node A's old parent should be pointing to node C */
    if(C->parent != NULL_NODE) {
      if(mNodes[C->parent].leftChild == node) {
        mNodes[C->parent].leftChild = iC;
      }
      else {
        mNodes[C->parent].rightChild = iC;
      }
    }
    else {
      mRoot = iC;
    }

    assert(!C->isLeaf());
    assert(!A->isLeaf());

    /* Height of node C was higher than B because of F */
    if(F->height > G->height) {
      C->rightChild = iF;
      A->rightChild = iG;
      G->parent = node;
      /* Need to recalculate height as well as the AABB */
      A->aabb.combine(B->aabb, G->aabb);
      C->aabb.combine(A->aabb, F->aabb);
      A->height = 1 + std::max(B->height, G->height);
      C->height = 1 + std::max(A->height, F->height);
      assert(A->height > 0);
      assert(C->height > 0);
    }
    /* Height of node C was higher than B because of G */
    else {
      C->rightChild = iG;
      A->rightChild = iF;
      F->parent = node;
      /* Need to recalculate height as well as the AABB */
      A->aabb.combine(B->aabb, F->aabb);
      C->aabb.combine(A->aabb, G->aabb);
      A->height = 1 + std::max(B->height, F->height);
      C->height = 1 + std::max(A->height, G->height);
      assert(A->height > 0);
      assert(C->height > 0);
    }

    /* New root of sub-tree */
    return iC;
  }

  /* Bubble B up ig higher than node C by 2 */
  if(balance < -1) {
    int32 iD = B->leftChild;
    int32 iE = B->rightChild;
    Node* D = mNodes + iD;
    Node* E = mNodes + iE;

    /* Swap node A and node B */
    B->leftChild = node;
    B->parent = A->parent;
    A->parent = iB;

    /* Node A's old parent should be pointing to node B */
    if(B->parent != NULL_NODE) {
      if(mNodes[B->parent].leftChild == node) {
        mNodes[B->parent].leftChild = iB;
      }
      else {
        mNodes[B->parent].rightChild = iB;
      }
    }
    else {
      mRoot = iB;
    }

    assert(!B->isLeaf());
    assert(!A->isLeaf());

    /* Height of node B was higher than C because of D */
    if(D->height > E->height) {
      B->rightChild = iD;
      A->leftChild = iE;
      E->parent = node;
      /* Need to recalculate height as well as the AABB */
      A->aabb.combine(C->aabb, E->aabb);
      B->aabb.combine(A->aabb, D->aabb);
      A->height = 1 + std::max(C->height, E->height);
      B->height = 1 + std::max(A->height, D->height);
      assert(A->height > 0);
      assert(B->height > 0);
    }
    /* Height of node B was higher than C because of E */
    else {
      B->rightChild = iE;
      A->leftChild = iD;
      D->parent = node;
      /* Need to recalculate height as well as the AABB */
      A->aabb.combine(C->aabb, D->aabb);
      B->aabb.combine(A->aabb, E->aabb);
      A->height = 1 + std::max(C->height, D->height);
      B->height = 1 + std::max(A->height, E->height);
    }

    /* New root of sub-tree */
    return iB;
  }

  return node;
}

/* Insert an object into the tree given its AABB */
int32 DynamicTree::insertObject(const AABB& aabb) {
  /* Next available node in the array */
  int32 node = createNode();

  /* Debug */
  /* Fat AABB to add into dynamic tree */
  const Vector2 padding(aabb.getHalfExtents() * mFatAABBInflation);
  mNodes[node].aabb.setLowerBound(aabb.getlowerBound() - padding);
  mNodes[node].aabb.setUpperBound(aabb.getUpperBound() + padding);
  /* Insert object as a leaf node */
  mNodes[node].height = LEAF_HEIGHT;
  insertLeaf(node);
  assert(mNodes[node].isLeaf());
  assert(node >= 0);
  return node;
}

/* Compute the height of the tree */
int32 DynamicTree::height() {
  return getNodeHeight(mRoot);
}

/* Get the root AABB */
AABB DynamicTree::getRootAABB() const {
  return getFatAABB(mRoot);
}

/* Get a node's enlarged AABB */
const AABB& DynamicTree::getFatAABB(int32 node) const {
  assert(node >= 0 && node < mNumAllocatedNodes);
  return mNodes[node].aabb;
}

/* Get data associated with the given node */
void* DynamicTree::getNodeData(int32 node) const {
  assert(node >= 0 && node < mNumAllocatedNodes);
  assert(mNodes[node].isLeaf());
  return mNodes[node].data;
}

/* Add an object into the tree */
int32 DynamicTree::add(const AABB& aabb, void* data) {
  int32 node = insertObject(aabb);
  mNodes[node].data = data;
  return node;
}

/* Remove an object from the tree */
void DynamicTree::remove(int32 node) {
  assert(node >= 0 && node < mNumAllocatedNodes);
  assert(mNodes[node].isLeaf());
  /* Remove the node from the tree and move to free nodes */
  removeLeaf(node);
  extractNode(node);
}

/* Update object when it has moved */
bool DynamicTree::update(int32 node, const AABB& aabb, bool forceInsert) {
  assert(node >= 0 && node < mNumAllocatedNodes);
  assert(mNodes[node].isLeaf());
  assert(mNodes[node].height >= 0);

  /* New AABB of collider is still inside the fat AABB of its node */
  if(!forceInsert && mNodes[node].aabb.contains(aabb)) {
    return false;
  }

  /* New AABB of collider is outside the fat AABB so remove the node from the tree */
  removeLeaf(node);
  mNodes[node].aabb = aabb;
  /* Debug */
  /* Compute the new AABB */
  const Vector2 padding(aabb.getHalfExtents() * mFatAABBInflation);
  mNodes[node].aabb.setLowerBound(aabb.getlowerBound() - padding);
  mNodes[node].aabb.setUpperBound(aabb.getUpperBound() + padding);
  assert(mNodes[node].aabb.contains(aabb));
  /* Now reinsert into the dynamic tree */
  insertLeaf(node);
  return true;
}

/* Get all of the shapes that are overlapping with the provided test shapes */
void DynamicTree::getShapeShapeOverlaps(const DynamicArray<int32>& testNodes, uint32 begin, uint32 end, DynamicArray<Pair<int32, int32>>& overlappingNodes) const {
  /* Stack of nodes to visit in tree traversal */
  Stack<int32> stack(mMemoryHandler);

  for(uint32 i = begin; i < end; i++) {
    stack.push(mRoot);
    const AABB& testAABB = getFatAABB(testNodes[i]);

    /* There are still nodes to be visited */
    while(!stack.empty()) {
      /* Next node to visit */
      const int32 visit = stack.pop();

      /* Disregard null nodes */
      if(visit == NULL_NODE) {
        continue;
      }

      /* Get the actual node pointer using the identifier obtained from the stack */
      const Node* visitNode = mNodes + visit;

      if(testAABB.isOverlapping(visitNode->aabb)) {
        /* If the two AABBs overlap and the visited node is a leaf, then we have found a unique pair of overlapping nodes */
        if(visitNode->isLeaf()) {
          overlappingNodes.add(Pair<int32, int32>(testNodes[i], visit));
        }
        /* Otherwise, we need to keep searching by visiting the children of the current node */
        else {
          stack.push(visitNode->leftChild);
          stack.push(visitNode->rightChild);
        }
      }
    }

    stack.clear();
  }
}

/* Get all of the shapes that are overlapping with the provided AABB */
void DynamicTree::getShapeAABBOverlap(const AABB& aabb, DynamicArray<int32>& overlappingNodes) const {
  /* Stack of nodes to visit in tree traversal */
  Stack<int32> stack(mMemoryHandler);
  stack.push(mRoot);

  /* There are still nodes to be visited */
  while(!stack.empty()) {
    /* Next node to visit */
    const int32 visit = stack.pop();

    /* Disregard null nodes */
    if(visit == NULL_NODE) {
      continue;
    }

    /* Get the actual node pointer using the identifier obtained from the stack */
    const Node* visitNode = mNodes + visit;

    if(aabb.isOverlapping(visitNode->aabb)) {
      /* If the two AABBs overlap and the visited node is a leaf, then we have found a unique pair of overlapping nodes */
      if(visitNode->isLeaf()) {
        overlappingNodes.add(visit);
      }
      /* Otherwise, we need to keep searching by visiting the children of the current node */
      else {
        stack.push(visitNode->leftChild);
        stack.push(visitNode->rightChild);
      }
    }
  }
}

/* Clear the tree */
void DynamicTree::clear() {
  /* Destroy all nodes */
  for(int32 i = 0; i < mNumAllocatedNodes; i++) {
    mNodes[i].~Node();
  }

  /* Free memory allocated */
  mMemoryHandler.free(mNodes, static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node));

  /* Re-initialize */
  initialize();
}
