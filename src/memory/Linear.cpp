#include <cstdlib>
#include <cassert>
#include <physics/memory/Linear.h>
#include <physics/memory/MemoryHandler.h>

using namespace physics;

LinearMemoryHandler::LinearMemoryHandler(MemoryHandler& primaryMemoryHandler) : mPrimaryMemoryHandler(primaryMemoryHandler), mOffset(0), mSize(INIT_SIZE) {
  /* Allocate INIT_SIZE memory */
  mStart = static_cast<char*>(mPrimaryMemoryHandler.allocate(mSize));
  assert(mStart != nullptr);
}

LinearMemoryHandler::~LinearMemoryHandler() {
  /* Free mSize memory beginning at mStart */
  mPrimaryMemoryHandler.free(mStart, mSize);
}

void* LinearMemoryHandler::allocate(size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);

  /* Not possible to allocate zero memory */
  if(size == 0) {
    return nullptr;
  }

  if(mOffset + size > mSize) {
    /* Vanilla allocation */
    return mPrimaryMemoryHandler.allocate(size);
  }

  /* Increase offset for new allocation */
  mOffset += size;

  /* Return requested memory block for user at mStart + mOffset of size bytes */
  return mStart + mOffset - size;
}

void LinearMemoryHandler::free(void* ptr, size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);

  /* Not posible to free zero memory */
  if(size == 0) {
    return;
  }

  char* char_ptr = static_cast<char*>(ptr);

  if(char_ptr < mStart ||
     char_ptr > mStart + mSize) {
      /* This memory would have been allocated if the memory space initially allocated was filled */
      mPrimaryMemoryHandler.free(char_ptr, size);
  }
}