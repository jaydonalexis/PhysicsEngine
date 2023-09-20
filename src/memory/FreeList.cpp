#include <cstdlib>
#include <cassert>
#include <physics/memory/FreeList.h>
#include <physics/memory/MemoryHandler.h>

using namespace physics;

size_t FreeListMemoryHandler::INIT_SIZE = 5242880;

/* Constructor */
FreeListMemoryHandler::FreeListMemoryHandler(MemoryHandler& primaryMemoryHandler, size_t initSize) : mPrimaryMemoryHandler(primaryMemoryHandler), mAllocated(0), mHead(nullptr), mFree(nullptr) {
  reserve(initSize == 0 ? INIT_SIZE : initSize);
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