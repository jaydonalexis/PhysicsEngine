#include <cstdlib>
#include <cassert>
#include <physics/memory/ObjectPool.h>
#include <physics/memory/MemoryHandler.h>

using namespace physics;

/* Static variables */
bool ObjectPoolMemoryHandler::init = false;
uint ObjectPoolMemoryHandler::mChunkSizes[NUM_POOL_GROUPS];
uint ObjectPoolMemoryHandler::mChunkSizePoolMap[MAX_CHUNK_SIZE + 1];

/* Constructor */
ObjectPoolMemoryHandler::ObjectPoolMemoryHandler(MemoryHandler& primaryMemoryHandler) : mPrimaryMemoryHandler(primaryMemoryHandler) {
  mNumAllocatedPools = 64;
  mNumUsedPools = 0;
  const size_t size = mNumAllocatedPools * sizeof(Pool);
  mPools = static_cast<Pool*>(primaryMemoryHandler.allocate(size));
  memset(mPools, 0, size);
  memset(mHeads, 0, sizeof(mHeads));

  if(!init) {
    /* Chunk sizes contains the range of all valid chunk sizes associated with the different pool groups */
    for(uint i = 0; i < NUM_POOL_GROUPS; i++) {
      mChunkSizes[i] = (i + 1) * 8;
    }

    /* Associate allocation sizes with pool groups */
    for(uint i = 0, j = 0; i <= MAX_CHUNK_SIZE; i++) {
      if(i == 0) {
        mChunkSizePoolMap[i] = -1;
        continue;
      }

      if(i <= mChunkSizes[j]) {
        mChunkSizePoolMap[i] = j;
      }
      else {
        j++;
        mChunkSizePoolMap[i] = j;
      }
    }

    init = true;
  }
}

/* Destructor */
ObjectPoolMemoryHandler::~ObjectPoolMemoryHandler() {
  /* Free the memory for all chunks in each of the pools */
  for(uint i = 0; i < mNumUsedPools; i++) {
    mPrimaryMemoryHandler.free(mPools[i].chunks, POOL_SIZE);
  }

  mPrimaryMemoryHandler.free(mPools, mNumAllocatedPools * sizeof(Pool));
}

/* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
void* ObjectPoolMemoryHandler::allocate(size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);

  /* Not possible to allocate zero memory */
  if(size == 0) {
    return nullptr;
  }

  /* Special case where the size to be allocated is more than the maximum size of a chunk */
  if(size > MAX_CHUNK_SIZE) {
    /* Base allocation */
    return mPrimaryMemoryHandler.allocate(size);
  }

  /* Find the pool group associated with the requested chunk size */
  int poolGroupIndex = mChunkSizePoolMap[size];
  assert(poolGroupIndex >= 0 && poolGroupIndex < NUM_POOL_GROUPS);

  /* A free chunk exists in the object pool */
  if(mHeads[poolGroupIndex]) {
    /* Return the next free chunk for the given pool group */
    Chunk* chunk = mHeads[poolGroupIndex];
    mHeads[poolGroupIndex] = chunk->nextChunk;
    return chunk;
  }
  else {
    /* More memory needs to be allocated for additional pools */
    if(mNumUsedPools == mNumAllocatedPools) {
      Pool* pools = mPools;
      mNumAllocatedPools += 64;
      mPools = static_cast<Pool*>(mPrimaryMemoryHandler.allocate(mNumAllocatedPools * sizeof(Pool)));
      memcpy(mPools, pools, mNumUsedPools * sizeof(Pool));
      memset(mPools + mNumUsedPools, 0, 64 * sizeof(Pool));
      mPrimaryMemoryHandler.free(pools, mNumUsedPools * sizeof(Pool));
    }

    /* Allocate a new pool */
    Pool* pool = mPools + mNumUsedPools;
    pool->chunks = static_cast<Chunk*>(mPrimaryMemoryHandler.allocate(POOL_SIZE));
    assert(pool->chunks);
    uint chunkSize = mChunkSizes[poolGroupIndex];
    assert(chunkSize <= MAX_CHUNK_SIZE);
    uint numChunks = (POOL_SIZE) / chunkSize;
    void* rawChunkHead = static_cast<void*>(pool->chunks);
    char* charChunkHead = static_cast<char*>(rawChunkHead);

    /* Divide the pool into individual chunks and link them */
    for(uint i = 0; i < numChunks; i++) {
      void* rawChunk;
      void* nextRawChunk;
      Chunk* chunk;
      Chunk* nextChunk;

      /* Special case for the last chunk in the pool where we assign a null pointer as the next chunk */
      if(i == numChunks - 1) {
        rawChunk = static_cast<void*>(charChunkHead + chunkSize * i);
        chunk = static_cast<Chunk*>(rawChunk);
        chunk->nextChunk = nullptr;
        continue;
      }
    
      /* For all other chunks, we assign a valid chunk as the next chunk in the sequence */
      rawChunk = static_cast<void*>(charChunkHead + chunkSize * i);
      nextRawChunk = static_cast<void*>(charChunkHead + chunkSize * (i + 1));
      chunk = static_cast<Chunk*>(rawChunk);
      nextChunk = static_cast<Chunk*>(nextRawChunk);
      chunk->nextChunk = nextChunk;
    }

    /* Update the head for the current pool group */
    mHeads[poolGroupIndex] = pool->chunks->nextChunk;
    mNumUsedPools++;

    return pool->chunks;
  }
}

/* Free dynamically allocated memory */
void ObjectPoolMemoryHandler::free(void* ptr, size_t size) {
  std::lock_guard<std::mutex> lock(mMutex);
  assert(size > 0);

  /* Not possible to free zero memory */
  if(size == 0) {
    return;
  }

  /* Special case where the size to be freed is more than the maximum size of a chunk */
  if(size > MAX_CHUNK_SIZE) {
    /* Free memory created using vanilla allocation */
    mPrimaryMemoryHandler.free(ptr, size);
    return;
  }

  /* Find the pool group associated with the requested chunk size */
  int poolGroupIndex = mChunkSizePoolMap[size];
  assert(poolGroupIndex >= 0 && poolGroupIndex < NUM_POOL_GROUPS);

  /* Clear and move the freed chunk to the first free chunk position in the pool group */
  Chunk* chunk = static_cast<Chunk*>(ptr);
  chunk->nextChunk = mHeads[poolGroupIndex];
  mHeads[poolGroupIndex] = chunk;
}