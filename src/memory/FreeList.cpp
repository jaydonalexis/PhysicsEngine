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

  /* Not possible to allocated zero memory */
  if(size == 0) {
    return nullptr;
  }

  AllocationHeader* block = mHead;
  assert(!mHead->prev);

  /* Check for a free block */
  if(mFree) {
    assert(!mFree->isAllocated);

    /* Can the block accomodate the size request */
    if(size <= mFree->size) {
      block = mFree;
      mFree = nullptr;
    }
  }

  while(block) {
    if(!block->isAllocated && size <= block->size) {
      split(block, size);
      break;
    }

    block = block->next;
  }

  if(!block) {
    apportion((mAllocated + size) * 2);
    assert(mFree);
    assert(!mFree->isAllocated);
    block = mFree;
    assert(block->size >= size);
    split(block, size);
  }

  block->isAllocated = true;

  /* Debug */
  if(block->next && !block->next->isAllocated) {
    mFree = block->next;
  }

  return static_cast<void*>(reinterpret_cast<unsigned char*>(block) + sizeof(AllocationHeader));
}

/* Free dynamically allocated memory */

/* Apportion additional memory */
void FreeListMemoryHandler::apportion(size_t size) {
  void* raw = mPrimaryMemoryHandler.allocate(size + sizeof(AllocationHeader));
  assert(raw);

  AllocationHeader* block = new (raw) AllocationHeader(size, nullptr, mHead, false);

  if(mHead) {
    mHead->prev = block;
  }

  mHead = block;
  mFree = mHead;
  mAllocated += size;
}