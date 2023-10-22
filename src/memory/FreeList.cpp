#include <cstdlib>
#include <cassert>
#include <physics/memory/FreeList.h>
#include <physics/memory/MemoryHandler.h>

using namespace physics;

size_t FreeListMemoryHandler::INIT_SIZE = 5242880;

/* Constructor */
FreeListMemoryHandler::FreeListMemoryHandler(MemoryHandler& primaryMemoryHandler, size_t initSize) : mPrimaryMemoryHandler(primaryMemoryHandler), mAllocated(0), mHead(nullptr), mFree(nullptr) {
  apportion(initSize == 0 ? INIT_SIZE : initSize);
}

/* Destructor */
FreeListMemoryHandler::~FreeListMemoryHandler() {
  AllocationHeader* block = mHead;

  while(block) {
    assert(!block->isAllocated);
    AllocationHeader* next = block->next;
    const size_t size = block->size;

    /* Release the memory allocated for each memory block */
    block->~AllocationHeader();
    mPrimaryMemoryHandler.free(static_cast<void*>(block), size + sizeof(AllocationHeader));
    block = next;
  }
}

/* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
void* FreeListMemoryHandler::allocate(size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);

  /* Not possible to allocate zero memory */
  if(size == 0) {
    return nullptr;
  }

  AllocationHeader* block = mHead;
  assert(!mHead->prev);

  /* Check for a free block */
  if(mFree) {
    assert(!mFree->isAllocated);

    /* Can the block accomodate the allocation request */
    if(size <= mFree->size) {
      block = mFree;
      mFree = nullptr;
    }
  }

  while(block) {
    /* Block can accomodate the allocation request */
    if(!block->isAllocated && size <= block->size) {
      /* Split memory block into two portions */
      split(block, size);
      break;
    }

    block = block->next;
  }

  /* More memory required */
  if(!block) {
    apportion((mAllocated + size) * 2);
    assert(mFree);
    assert(!mFree->isAllocated);
    /* Request for extra memory has been fulfilled */
    block = mFree;
    assert(block->size >= size);
    split(block, size);
  }

  block->isAllocated = true;

  /* Debug */
  if(block->next && !block->next->isAllocated) {
    mFree = block->next;
  }

  /* Void pointer to the memory space of the block after the allocation header */
  return static_cast<void*>(reinterpret_cast<unsigned char*>(block) + sizeof(AllocationHeader));
}

/* Free dynamically allocated memory */
void FreeListMemoryHandler::free(void* ptr, size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);
  
  /* Not possible to free zero memory */
  if(size == 0) {
    return;
  }

  unsigned char* blockAddress = static_cast<unsigned char*>(ptr) - sizeof(AllocationHeader);
  AllocationHeader* block = reinterpret_cast<AllocationHeader*>(blockAddress);
  assert(block->isAllocated);
  block->isAllocated = false;

  AllocationHeader* tempBlock = block;

  /* Merge previous block with current block if contiguous in memory and the previous block is not allocated */
  if(block->prev && !block->prev->isAllocated && block->prev->isNextCoalescent) {
    tempBlock = block->prev;
    coalesce(block->prev, block);
  }
  
  /* Merge next block with current block is contiguous in memory and the next block is not allocated */
  if(tempBlock->next && !tempBlock->next->isAllocated && tempBlock->isNextCoalescent) {
    coalesce(tempBlock, tempBlock->next);
  }

  mFree = tempBlock;
}

/* Split memory block into two portions */
void FreeListMemoryHandler::split(AllocationHeader* block, size_t size) {
  assert(size <= block->size);
  assert(!block->isAllocated);

  if(size + sizeof(AllocationHeader) < block->size) {
    /* New memory block for the leftover space */
    unsigned char* nextBlockAddress = (reinterpret_cast<unsigned char*>(block)) + sizeof(AllocationHeader) + size;
    AllocationHeader* newBlock = new (static_cast<void*>(nextBlockAddress)) AllocationHeader(block->size - sizeof(AllocationHeader) - size, block, block->next, block->isNextCoalescent);
    assert(newBlock->next != newBlock);
    block->next = newBlock;

    if(newBlock->next) {
      newBlock->next->prev = newBlock;
    }

    assert(block->next != block);
    block->isNextCoalescent = true;
    block->size = size;
    assert(!block->prev || block->prev->next == block);
    assert(!block->next || block->next->prev == block);
    assert(!newBlock->prev || newBlock->prev->next == newBlock);
    assert(!newBlock->next || newBlock->next->prev == newBlock);
  }
}

/* Coalesce two blocks */
void FreeListMemoryHandler::coalesce(AllocationHeader* block1, AllocationHeader* block2) {
  assert(block1->next == block2);
  assert(block2->prev == block1);
  assert(!block1->isAllocated);
  assert(!block2->isAllocated);
  assert(block1->isNextCoalescent);

  block1->size += sizeof(AllocationHeader) + block2->size;
  block1->next = block2->next;
  assert(block1->next != block1);

  if(block2->next) {
    block2->next->prev = block1;
  }

  block1->isNextCoalescent = block2->isNextCoalescent;
  block2->~AllocationHeader();
  assert(!block1->prev || block1->prev->next == block1);
  assert(!block1->next || block1->next->prev == block1);
}

/* Apportion additional memory */
void FreeListMemoryHandler::apportion(size_t size) {
  /* Call on primary memory handler to allocate new memory */
  void* raw = mPrimaryMemoryHandler.allocate(size + sizeof(AllocationHeader));
  assert(raw);

  /* New header for our allocated memory */
  AllocationHeader* block = new (raw) AllocationHeader(size, nullptr, mHead, false);

  if(mHead) {
    mHead->prev = block;
  }

  /* Allocated memory added to the beginning of the linked list */
  mHead = block;
  mFree = mHead;
  mAllocated += size;
}