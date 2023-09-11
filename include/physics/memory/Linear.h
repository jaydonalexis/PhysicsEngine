#ifndef PHYSICS_LINEAR_HANDLER_H
#define PHYSICS_LINEAR_HANDLER_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <mutex>

namespace physics {

class LinearMemoryHandler : public MemoryHandler {

  private:
    /* -- Attributes -- */

    /* Initial size */
    size_t INIT_SIZE = 1048576;

    /* Size of memory space */
    size_t mSize;

    /* Mutex to prevent simultaneous allocations */
    std::mutex mMutex;

    /* Primary memory handler */
    MemoryHandler& mPrimaryMemoryHandler;

    /* Start pointer of memory space */
    char* mStart;

    /* Offset in bytes from the starting address */
    size_t mOffset;

  public:
    /* Methods */

    /* Constructor */
    LinearMemoryHandler(MemoryHandler& primaryMemoryHandler);

    /* Destructor */
    ~LinearMemoryHandler() override;

    /* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
    void* allocate(size_t size) override;

    /* Free dynamically allocated memory */
    void free(void* ptr, size_t size) override;

    /* Reset pointer to mStart */
    void reset();
};

}

#endif