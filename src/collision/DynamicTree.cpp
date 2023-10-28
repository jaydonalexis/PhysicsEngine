#include <physics/collision/DynamicTree.h>
#include <physics/collections/Stack.h>

using namespace physics;

/* Constructor */
DynamicTree::DynamicTree(MemoryHandler& memoryHandler, float fatAABBInflation) : mMemoryHandler(memoryHandler), mFatAABBInflation(fatAABBInflation) {
  initialize();
}

/* Destructor */
DynamicTree::~DynamicTree() {
  mMemoryHandler.free(mNodes, static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node));
}

/* Initialization function */
void DynamicTree::initialize() {
  mRoot = NULL_NODE;
  mNumNodes = 0;
  mNumAllocatedNodes = 8;

  /* Allocate, construct and initialize nodes */
  mNodes = static_cast<Node*>(mMemoryHandler.allocate(static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node)));

  for(int32 i = 0; i < mNumAllocatedNodes; i++) {
    new (mNodes + i) Node();

    if(i == mNumAllocatedNodes - 1) {
      mNodes[i].next = NULL_NODE;
      mNodes[i].height = FREE_NODE_HEIGHT;
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

  int32 heightLeft = getNodeHeight(root->leftChild);
  int32 heightRight = getNodeHeight(root->rightChild);
  return 1 + std::max(heightLeft, heightRight);
}

/* Allocate a node */
int32 DynamicTree::createNode() {
  if(mFree == NULL_NODE) {
    int32 numAllocatedNodesPrev = mNumAllocatedNodes;
    mNumAllocatedNodes *= 2;
    Node* nodesPrev = mNodes;
    mNodes = static_cast<Node*>(mMemoryHandler.allocate(static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node)));
    std::uninitialized_copy(nodesPrev, nodesPrev + mNumNodes, mNodes);
    mMemoryHandler.free(nodesPrev, static_cast<size_t>(numAllocatedNodesPrev) * sizeof(Node));

    /* Init newly allocated nodes */
    for(int32 i = 0; i < mNumAllocatedNodes; i++) {
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
  return free;
}

/* Release a node (this does not mean deallocation) */
void DynamicTree::extractNode(int32 node) {
  mNodes[node].next = mFree;
  mNodes[node].height = FREE_NODE_HEIGHT;
  mFree = node;
  mNumNodes--;
}

/* Insert a node as a leaf in the tree */
void DynamicTree::insertLeaf(int32 node) {
  if(mRoot == NULL_NODE) {
    mRoot = node;
    mNodes[mRoot].parent = NULL_NODE;
    return;
  }

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
    AABB aabb;
    aabb.combine(leafAABB, mNodes[leftChild].aabb);

    /* If the node is a leaf node, we replace it with a new node P and make C and N the children of P */
    if(mNodes[leftChild].isLeaf()) {
      /* Note that combined area is not doubled here due to the fact that this is a case where we visit a specific child of S */
      costLeft = aabb.getPerimeter() + costInheritance;
    }
    /* Otherwise, the cost is at least the following */
    else {
      costLeft = aabb.getPerimeter() - mNodes[leftChild].aabb.getPerimeter() + costInheritance;
    }

    /* Cost of descending into right child */
    float costRight;
    aabb.combine(leafAABB, mNodes[rightChild].aabb);

    /* If the node is a leaf node, we replace it with a new node P and make C and N the children of P */
    if(mNodes[rightChild].isLeaf()) {
      /* Note that combined area is not doubled here due to the fact that this is a case where we visit a specific child of S */
      costRight = aabb.getPerimeter() + costInheritance;
    }
    /* Otherwise, the cost is at least the following */
    else {
      costRight = aabb.getPerimeter() - mNodes[rightChild].aabb.getPerimeter() + costInheritance;
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

  if(parentPrev != NULL_NODE) {
    if(mNodes[parentPrev].leftChild == sibling) {
      mNodes[parentPrev].leftChild = parent;
    }
    else {
      mNodes[parentPrev].rightChild = parent;
    }
  }
  else {
    mRoot = parent;
  }

  mNodes[parent].leftChild = sibling;
  mNodes[parent].rightChild = node;
  mNodes[sibling].parent = parent;
  mNodes[node].parent = parent;

  walk = mNodes[node].parent;

  /* Now that the node is inserted, we need to propogate corrective measures up the tree */
  while(walk != NULL_NODE) {
    walk = balance(walk);
    int32 leftChild = mNodes[walk].leftChild;
    int32 rightChild = mNodes[walk].rightChild;
    mNodes[walk].height = 1 + std::max(mNodes[leftChild].height, mNodes[rightChild].height);
    mNodes[walk].aabb.combine(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
    walk = mNodes[walk].parent;
  }
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
      walk = balance(walk);
      int32 leftChild = mNodes[walk].leftChild;
      int32 rightChild = mNodes[walk].rightChild;
      mNodes[walk].height = 1 + std::max(mNodes[leftChild].height, mNodes[rightChild].height);
      mNodes[walk].aabb.combine(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
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
  Node* A = mNodes + node;
  
  if(A->isLeaf() || A->height < 2) {
    return node;
  }

  int32 iB = A->leftChild;
  int32 iC = A->rightChild;
  Node* B = mNodes + iB;
  Node* C = mNodes + iC;
  int32 balance = C->height - B->height;

  /* Bubble C up */
  if(balance > 1) {
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

    if(F->height > G->height) {
      C->rightChild = iF;
      A->rightChild = iG;
      G->parent = node;
      A->aabb.combine(B->aabb, G->aabb);
      C->aabb.combine(A->aabb, F->aabb);
      A->height = 1 + std::max(B->height, G->height);
      C->height = 1 + std::max(A->height, F->height);
    }
    else {
      C->rightChild = iG;
      A->rightChild = iF;
      G->parent = node;
      A->aabb.combine(B->aabb, F->aabb);
      C->aabb.combine(A->aabb, G->aabb);
      A->height = 1 + std::max(B->height, F->height);
      C->height = 1 + std::max(A->height, G->height);
    }

    return iC;
  }

  /* Bubble B up */
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

    if(D->height > E->height) {
      B->rightChild = iD;
      A->leftChild = iE;
      E->parent = node;
      A->aabb.combine(C->aabb, E->aabb);
      B->aabb.combine(A->aabb, D->aabb);
      A->height = 1 + std::max(C->height, E->height);
      B->height = 1 + std::max(A->height, D->height);
    }
    else {
      B->rightChild = iE;
      A->leftChild = iD;
      D->parent = node;
      A->aabb.combine(C->aabb, D->aabb);
      B->aabb.combine(A->aabb, E->aabb);
      A->height = 1 + std::max(C->height, D->height);
      B->height = 1 + std::max(A->height, E->height);
    }

    return iB;
  }

  return node;
}

/* Insert an object into the tree given its AABB */
int32 DynamicTree::insertObject(const AABB& aabb) {
  int32 node = createNode();
  assert(node >= 0);

  /* Fat AABB */
  /* Debug */
  const Vector2 padding(aabb.getHalfExtents() * 0.5f);
  mNodes[node].aabb.setLowerBound(aabb.getlowerBound() - padding);
  mNodes[node].aabb.setUpperBound(aabb.getUpperBound() + padding);
  mNodes[node].height = LEAF_HEIGHT;
  insertLeaf(node);
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
  removeLeaf(node);

  /* Move to free nodes */
  extractNode(node);
}

/* Update object when it has moved */
bool DynamicTree::update(int32 node, const AABB& aabb, bool force) {
  assert(node >= 0 && node < mNumAllocatedNodes);
  assert(mNodes[node].isLeaf());

  if(!force && mNodes[node].aabb.contains(aabb)) {
    return false;
  }

  removeLeaf(node);
  mNodes[node].aabb = aabb;
  /* Debug */
  const Vector2 padding(aabb.getHalfExtents() * mFatAABBInflation);
  mNodes[node].aabb.setLowerBound(aabb.getlowerBound() - padding);
  mNodes[node].aabb.setUpperBound(aabb.getUpperBound() + padding);
  assert(mNodes[node].aabb.contains(aabb));
  insertLeaf(node);
  return true;
}

/* Get all of the shapes that are overlapping with the provided test shapes */
void DynamicTree::getShapeShapeOverlaps(const DynamicArray<int32>& testNodes, uint32 begin, uint32 end, DynamicArray<Pair<int32, int32>>& overlappingNodes) const {
  Stack<int32> stack(mMemoryHandler);

  for(uint32 i = begin; i < end; i++) {
    stack.push(mRoot);
    const AABB& testAABB = getFatAABB(testNodes[i]);

    while(!stack.empty()) {
      const int32 visit = stack.pop();

      if(visit == NULL_NODE) {
        continue;
      }

      const Node* visitNode = mNodes + visit;

      if(testAABB.isOverlapping(visitNode->aabb)) {
        if(visitNode->isLeaf()) {
          overlappingNodes.add(Pair<int32, int32>(testNodes[i], visit));
        }
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
  Stack<int32> stack(mMemoryHandler);
  stack.push(mRoot);

  while(!stack.empty()) {
    const int32 visit = stack.pop();

    if(visit == NULL_NODE) {
      continue;
    }

    const Node* visitNode = mNodes + visit;

    if(aabb.isOverlapping(visitNode->aabb)) {
      if(visitNode->isLeaf()) {
        overlappingNodes.add(visit);
      }
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
