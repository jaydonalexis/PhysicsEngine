#ifndef PHYSICS_FREE_LIST_H
#define PHYSICS_FREE_LIST_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <mutex>

namespace physics {

class FreeListMemoryHandler : public MemoryHandler {

  private:
    /* -- Nested Classes -- */

    /* Allocation headers are used to record the size of the memory block needed to be freed when deallocating */
    struct AllocationHeader {

      public:
        /* -- Attributes -- */

        /* Size of allocated memory block */
        size_t size;

        /* Memory block's allocation status */
        bool isAllocated;

        /* Next memory block in list */
        AllocationHeader* next;

        /* Previous memory block in list */
        AllocationHeader* prev;

        /* Next memory block in the list is a valid candidate for coalescence */
        bool isNextCoalescent;

        /* -- Methods -- */

        /* Constructor */
        AllocationHeader(size_t size, AllocationHeader* prev, AllocationHeader* next, bool isNextCoalescent) : size(size), isAllocated(false), prev(prev), next(next), isNextCoalescent(isNextCoalescent) {
          assert(size > 0);
        }
    };

    /* -- Attributes -- */

    /* Initial size */
    static size_t INIT_SIZE;

    /* Mutex to prevent simultaneous allocations */
    std::mutex mMutex;

    /* Primary memory handler */
    MemoryHandler& mPrimaryMemoryHandler;

    /* Total size of allocated memory */
    size_t mAllocated;

    /* First block of the linked list */
    AllocationHeader* mHead;

    /* Free memory block */
    AllocationHeader* mFree;

    /* -- Methods -- */
    
    /* Split memory block into two portions */
    void split(AllocationHeader* block, size_t size);

    /* Coalesce two blocks */
    void coalesce(AllocationHeader* block1, AllocationHeader* block2);

    /* Apportion additional memory */
    void apportion(size_t size);

  public:
    /* -- Methods -- */

    /* Constructor */
    FreeListMemoryHandler(MemoryHandler& primaryMemoryHandler, size_t initSize = 0);

    /* Destructor */
    ~FreeListMemoryHandler() override;

    /* Overloaded assignment operator */
    FreeListMemoryHandler& operator=(FreeListMemoryHandler& memoryHandler) = delete;

    /* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
    void* allocate(size_t size) override;

    /* Free dynamically allocated memory */
    void free(void* ptr, size_t size) override;
};

}

#endif