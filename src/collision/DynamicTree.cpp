#include <physics/collision/DynamicTree.h>
#include <physics/collections/Stack.h>

using namespace physics;

/* Constructor */
DynamicTree::DynamicTree(MemoryHandler& memoryHandler, float fatAABBInflation) : mMemoryHandler(memoryHandler), mFatAABBInflation(fatAABBInflation) {
  init();
}

/* Destructor */
DynamicTree::~DynamicTree() {
  mMemoryHandler.free(mNodes, static_cast<size_t>(mNumAllocatedNodes) * sizeof(Node));
}

/* Initialization function */
void DynamicTree::init() {
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