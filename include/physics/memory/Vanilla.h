#ifndef PHYSICS_VANILLA_H
#define PHYSICS_VANILLA_H

#include <physics/memory/MemoryHandler.h>
#include <cstdlib>

namespace physics {

class VanillaMemoryHandler : public MemoryHandler {

  public:
    /* -- Methods -- */

    /* Destructor */
    ~VanillaMemoryHandler() override = default;

    /* Dynamically allocate memory of size in bytes and return a pointer to the heap allocated block */
    void* allocate(size_t size) override {
      return std::malloc(size);
    }

    /* Free dynamically allocated memory */
    void free(void* ptr, size_t) override {
      std::free(ptr);
    }
};

}

#endif

