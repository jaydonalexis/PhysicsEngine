#include <cstdlib>
#include <cassert>
#include <physics/memory/Linear.h>
#include <physics/memory/MemoryHandler.h>

using namespace physics;

/* Constructor */
LinearMemoryHandler::LinearMemoryHandler(MemoryHandler& primaryMemoryHandler) : mPrimaryMemoryHandler(primaryMemoryHandler), mOffset(0), mSize(INIT_SIZE), mNumValidShrinkFrames(0), mGrow(false) {
  /* Allocate INIT_SIZE memory */
  mStart = static_cast<char*>(mPrimaryMemoryHandler.allocate(mSize));
  assert(mStart);
}

/* Destructor */
LinearMemoryHandler::~LinearMemoryHandler() {
  /* Free mSize memory beginning at mStart */
  mPrimaryMemoryHandler.free(mStart, mSize);
}

/* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
void* LinearMemoryHandler::allocate(size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);

  if(mOffset + size > mSize) {
    /* Vanilla allocation */
    mGrow = true;
    return mPrimaryMemoryHandler.allocate(size);
  }

  /* Next available memory location */
  void* raw = mStart + mOffset;

  /* Increase offset for new allocation */
  mOffset += size;

  /* Return requested memory block for user at mStart + mOffset of size bytes */
  return raw;
}

/* Free dynamically allocated memory */
void LinearMemoryHandler::free(void* ptr, size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);

  char* char_ptr = static_cast<char*>(ptr);

  if(char_ptr < mStart ||
     char_ptr > mStart + mSize) {
      /* This memory would have been allocated if the memory space initially allocated was filled */
      mPrimaryMemoryHandler.free(char_ptr, size);
  }
}

/* Reset pointer to mStart */
void LinearMemoryHandler::reset() {
  std::lock_guard<std::mutex> lock(mMutex);

  if(mOffset < mSize / 2) {
    mNumValidShrinkFrames++;

    if(mNumValidShrinkFrames > NUM_FRAMES_BEFORE_SHRINK) {
      /* Release memory */
      mPrimaryMemoryHandler.free(mStart, mSize);

      /* Reduce the allocation size for the next frame */
      mSize /= 2;

      if(!mSize) {
        mSize = 1;
      }

      /* Allocate memory */
      mStart = static_cast<char*>(mPrimaryMemoryHandler.allocate(mSize));
      assert(mStart);
      mNumValidShrinkFrames = 0;
    }
  }
  else {
    mNumValidShrinkFrames = 0;
  }

  if(mGrow) {
    /* Free the current memory space, multiply the total size by 2 and reallocate */
    mPrimaryMemoryHandler.free(mStart, mSize);
    mSize *= 2;
    mStart = static_cast<char*>(mPrimaryMemoryHandler.allocate(mSize));
    assert(mStart);
    mGrow = false;
    mNumValidShrinkFrames = 0;
  }

  /* Reset the offset so that it points to the beginning of the memory space */
  mOffset = 0;
}