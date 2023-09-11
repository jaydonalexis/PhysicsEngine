#ifndef PHYSICS_OBJECT_POOL_H
#define PHYSICS_OBJECT_POOL_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <mutex>

namespace physics {

class ObjectPoolMemoryHandler : public MemoryHandler {

  private:
    /* -- Nested Classes -- */

    /* Chunks are of predefined size and are the smallest units of memory in object pool allocation */
    struct Chunk {

      public:
        /* -- Attributes -- */

        /* Pointer to the next chunk within a given pool */
        Chunk* nextChunk;
    };

    struct Pool {

      public:
        /* -- Attributes -- */

        /* Pointer to the first chunk in the pool */
        Chunk* chunks;
    };

    /* -- Attributes -- */

    /* Maximum chunk size */
    static const size_t MAX_CHUNK_SIZE = 1024;

    /* Pool size */
    static const size_t POOL_SIZE = 16 * MAX_CHUNK_SIZE;

    /* Number of pool size groups */
    static const uint NUM_POOL_GROUPS = 128;

    /* Chunk sizes associated with a specific pool group */
    static uint mChunkSizes[NUM_POOL_GROUPS];

    /* Associate allocation sizes with pool groups */
    static uint mChunkSizeToPool[MAX_CHUNK_SIZE + 1];

    /* Pools initialized */
    static bool init;

    /* Mutex to prevent simultaneous allocations */
    std::mutex mMutex;

    /* Primary memory handler */
    MemoryHandler& mPrimaryMemoryHandler;

    /* Chunk at head for each pool group */
    Chunk* mHeads[NUM_POOL_GROUPS];

    /* Allocated Pools */
    Pool* mPools;

    /* Number of allocated pools */
    uint mNumAllocatedPools;

    /* Number of used pools */
    uint mNumUsedPools;

  public:
    /* -- Methods -- */

    /* Constructor */
    ObjectPoolMemoryHandler(MemoryHandler& primaryMemoryHandler);

    /* Destructor */
    ~ObjectPoolMemoryHandler() override;

    /* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
    void* allocate(size_t size) override;

    /* Free dynamically allocated memory */
    void free(void* ptr, size_t size) override;
};

}

#endif