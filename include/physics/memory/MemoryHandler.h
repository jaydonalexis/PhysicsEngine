#ifndef PHYSICS_MEMORY_HANDLER_H
#define PHYSICS_MEMORY_HANDLER_H

namespace physics {

class MemoryHandler {

  public:
    /* -- Methods -- */

    /* Constructor */
    MemoryHandler() = default;

    /* Destructor */
    virtual ~MemoryHandler() = default;

    /* Overloaded assignment operator */
    MemoryHandler& operator=(MemoryHandler& memoryHandler) = delete;
    
    /* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
    virtual void* allocate(size_t size) = 0;

    /* Free dynamically allocated memory */
    virtual void free(void* ptr, size_t size) = 0;
};

}

#endif