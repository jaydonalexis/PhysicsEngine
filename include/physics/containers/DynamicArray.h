#ifndef PHYSICS_DYNAMIC_ARRAY_H
#define PHYSICS_DYNAMIC_ARRAY_H

#include <physics/Configuration.h>
#include <cassert>
#include <cstring>
#include <iterator>
#include <memory>

namespace physics {

template<typename T>
class DynamicArray {

  private:
    /* -- Attributes -- */

    /* Array elements */
    T* mValues;
    
    /* Number of elements in the array */
    uint64 mSize;

    /* Number of allocated elements in the array */
    uint64 mCapacity;

    /* Memory Allocator */
    MemoryAllocator& mAllocator;
};

}

#endif