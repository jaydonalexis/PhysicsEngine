#ifndef PHYSICS_DYNAMIC_ARRAY_H
#define PHYSICS_DYNAMIC_ARRAY_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
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

    /* Memory Handler */
    MemoryHandler& mMemoryHandler;

  public:
    /* -- Traits -- */
    using iterator = T*;
    using const_iterator = const T*;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    /* -- Constructors -- */
    DynamicArray(MemoryHandler& memoryHandler, uint64 capacity = 0);
    DynamicArray(const DynamicArray<T>& array);

    /* -- Destructor -- */
    ~DynamicArray();

    /* -- Element Access -- */
    const T& front() const;
    T& front();
    const T& back() const;
    T& back();
    T& at(uint64 i);
    const T& at(uint64 i) const;

    /* -- Iterators -- */
    iterator begin() noexcept;
    const_iterator begin() const noexcept;
    iterator end() noexcept;
    const_iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const_iterator  cend() const;
    reverse_iterator rbegin() noexcept;
    const_reverse_iterator crbegin() const noexcept;
    reverse_iterator rend() noexcept;
    const_reverse_iterator crend() const noexcept;

    /* -- Capacity -- */
    uint64 size() const noexcept;
    uint64 capacity() const noexcept;
    constexpr bool empty() const noexcept;
    void reserve(uint64 size);

    /* -- Operators -- */
    T& operator[](const uint64 i);
    const T& operator[](const uint64 i) const;
    bool operator==(const DynamicArray<T>& array) const;
    bool operator!=(const DynamicArray<T>& array) const;
    DynamicArray<T>& operator=(const DynamicArray<T>& array);

    /* -- Modifiers -- */
    template<typename...Ts>
    void emplace(Ts&&... args);
    void add(const T& element);
    void add(const DynamicArray<T>& array, uint64 start = 0);
    iterator remove(const T& element);
    iterator remove(const iterator& iter);
    iterator removeAt(uint64 i);
    void grow(uint64 numElements);
    void clear(bool free = false);
};

}

#endif